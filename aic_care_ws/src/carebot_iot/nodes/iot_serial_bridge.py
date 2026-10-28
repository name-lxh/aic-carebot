#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
iot_serial_bridge.py — 智慧养老护理赛题：串口物联网网关桥接（备用通道）

为什么需要这个节点
------------------
主通道是 HomeAssistant REST API（iot_bridge.py）。但赛题明确要求
「比赛所选用的均为实际电器用品，不得使用模拟设备」，而 HomeAssistant 路径依赖：
  * 盒子在线、IP 未变
  * 长期令牌未失效
  * Zigbee 网络通畅

任何一环出问题，30 分的物联网任务就全丢。因此本节点提供**第二条独立硬件通道**：
直接驱动组委会配备的物联网串口网关（`/dev/IOT_net`，9600 波特），
其协议已从参考实现 `iot_modules/src/IOT_net.cpp` 逆向确认。

⚠️ 本节点是对参考实现的**重写**，修掉了原版的严重缺陷
------------------------------------------------------
原 `IOT_net.cpp` 的问题：
  1. **每次写串口都新建 `serial::Serial` 对象** —— 反复打开关闭设备，
     在高频控制下会失败或丢数据。
  2. **主循环没有 `loop_rate.sleep()`**（被注释掉了）—— 满核空转，
     在迷你主机上会抢占其他节点的 CPU。
  3. **把控制逻辑硬编码在 `else if` 链里**，不可配置、不可测试。

本节点的做法：
  * 串口对象只打开一次，复用。
  * 明确的轮询节奏（默认 5Hz）。
  * 协议封装成 `write_module()` / `read_module()`，可单独测试。
  * 设备/地址映射配置化。

协议（21 字节定长帧，小端 float）
--------------------------------
  写： 0xA5 | ADDR | CMD | float×4（各 4 字节 LE）| 0x0D | 0x0A
  读： 同结构；0xA5 0xA5 0xA5 表示 0xA5 字节（Python 读入为有符号）

模块地址表（来自 IOT_net.cpp 的实际使用）
----------------------------------------
  ADDR=1  继电器组： 数据1 = 风扇 (1=关, 2=开)
                     数据2 = 电灯 (1=关, 2=开)
  ADDR=2  环境传感： 数据1 = 光照, 数据2 = 温度, 数据3 = 湿度, 数据4 = CO₂
  ADDR=3  语音提醒（吃药提醒）
  ADDR=4  安防传感： 数据1 = 燃气 (1=正常, 2=泄漏)
                     数据2 = 漏水 (1=正常, 2=已发生)
  ADDR=9  窗帘继电器：数据2 = 窗帘 (1=关, 2=开)

订阅 / 发布
-----------
  /iot/cmd        std_msgs/String   与 iot_bridge 相同的 JSON 指令接口
  /iot/serial/env std_msgs/String   串口读到的环境数据 JSON
  /iot/serial/status std_msgs/String  人类可读状态
  /iot/serial/query std_srvs/Trigger  主动查询指定模块（message = "2" / "4"）

参数
----
  ~port          默认 /dev/IOT_net
  ~baudrate      默认 9600
  ~poll_interval 默认 0.2    主动读传感器周期（秒）；<=0 表示只按需读取
  ~read_timeout  默认 0.5    单次读超时（秒）
  ~enabled       默认 true
"""

import sys
import json
import struct
import threading

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

try:
    import serial
except ImportError:
    serial = None

try:
    unicode  # noqa: F821  (Python 2)
except NameError:
    unicode = str  # noqa: A001


def setup_stdout():
    """让 stdout/stderr 能输出中文 —— Python 2.7 / 3.6 / 3.11 都能用。

    ⚠️ 这段代码修正了 4 次才对，动它之前请看完下面每一条：

    1) 不用 `hasattr(sys.stdout, 'reconfigure')` 判断 —— 那是 3.7+ 的方法。
       目标环境 Ubuntu 18.04 是 Python 3.6，hasattr 为假，设置被静默跳过，
       随后打印中文抛 UnicodeEncodeError。

    2) 不能只依赖 `stream.buffer` —— 那是 Python 3 的属性。
       Python 2.7（ROS Melodic 的实际运行版本）的 sys.stdout 没有它，
       getattr 得 None，同样静默跳过。

    3) 用 `codecs.getwriter` 包 Python 2 的 sys.stdout 时，**必须**处理
       字节串输入。因为 Python 2 里有两类代码往 stderr 写**字节串**：

           traceback.print_exc()     → file.write(str)  字节串
           logging.StreamHandler     → stream.write(msg + "\n")

       标准 StreamWriter 只接受 unicode，收到字节串会先 decode('ascii')，
       遇到非 ASCII 就崩：
           UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5
       后果是"为了看清异常而打印异常时崩掉"，真正的错误彻底看不到。
       —— 本项目实际踩过这个坑，排查了很久。

    4) 所以这里自定义 _Utf8Writer：字节串原样写，unicode 编码后写。

    实测环境：Python 3.11（本地开发）/ 3.6.9（Ubuntu 18.04 默认）
              / 2.7.17（ROS Melodic 跑节点的实际版本）
    """
    import codecs as _codecs

    class _Utf8Writer(_codecs.StreamWriter):
        """字节串与 unicode 都接受，都往底层流写 UTF-8 字节。"""

        def write(self, s):
            if s is None:
                return
            try:
                if isinstance(s, bytes):
                    # 已经是字节串（traceback / logging 会这样写）
                    self.stream.write(s)
                    return
                # unicode → UTF-8 字节
                self.stream.write(s.encode('utf-8', 'backslashreplace'))
            except Exception:
                # 兜底：尽全力输出，绝不因为编码问题让程序崩
                try:
                    self.stream.write(repr(s))
                except Exception:
                    pass

    def _wrap(stream):
        if stream is None:
            return stream
        # 已经是 UTF-8 的，不动
        enc = (getattr(stream, 'encoding', None) or '').lower().replace('-', '')
        if enc in ('utf8', 'utf_8'):
            return stream
        # 优先用底层字节流（Python 3）；Python 2 没有 .buffer，直接用流本身
        buf = getattr(stream, 'buffer', None)
        if buf is None:
            buf = stream
        try:
            return _Utf8Writer(buf, errors='backslashreplace')
        except Exception:
            pass
        # 退化：Python 3.7+ 的 reconfigure
        reconf = getattr(stream, 'reconfigure', None)
        if callable(reconf):
            try:
                reconf(encoding='utf-8', errors='backslashreplace')
            except Exception:
                pass
        return stream

    sys.stdout = _wrap(sys.stdout)
    sys.stderr = _wrap(sys.stderr)


def fix_ros_logging():
    """修复 ROS 日志打印中文崩溃的问题（Python 2 下必现）。

    背景
    ----
    ROS Melodic 跑在 Python 2.7 下，Python 2 的 `str(unicode)` 默认用
    **ASCII** 编码。而 ROS 的日志处理器就是这么转的：

        rosgraph/roslogging.py:249
            msg = msg.replace('${message}', str(record_message))

    于是
        rospy.loginfo(u'中文')
    直接抛
        UnicodeEncodeError: 'ascii' codec can't encode characters

    **任何节点用 rospy.log* 打中文都会崩**，而且常常崩在异常处理路径上，
    把真正的异常掩盖掉，调试时极具误导性。

    解法
    ----
    在 record 进入 handler **之前**，把 unicode 消息转成 UTF-8 字节串。
    这样 roslogging 里的 `str(record_message)` 变成"字节串转字节串"，
    是无操作，不会再触发 ASCII 编码。

    ⚠️ 这段代码必须保留 —— 去掉后所有中文日志都会崩。
    """
    import logging as _logging

    # 避免重复打补丁
    if getattr(_logging.Logger, '_ros_utf8_patched', False):
        return

    try:
        _text_types = (unicode,)                      # noqa: F821  (Py2)
    except NameError:
        _text_types = (str,)                          # Py3

    _orig_makeRecord = _logging.Logger.makeRecord

    def _utf8_makeRecord(self, name, level, fn, lno, msg, args, exc_info,
                         func=None, extra=None, sinfo=None):
        # 把 unicode 消息转成 UTF-8 字节串
        if isinstance(msg, _text_types):
            try:
                msg = msg.encode('utf-8')
            except Exception:
                pass
        # 参数里的 unicode 也要转，否则 % 格式化时同样会崩
        if args:
            try:
                args = tuple(
                    a.encode('utf-8') if isinstance(a, _text_types) else a
                    for a in args)
            except Exception:
                pass
        try:
            return _orig_makeRecord(self, name, level, fn, lno, msg, args,
                                    exc_info, func, extra, sinfo)
        except TypeError:
            # Python 2.7 的签名没有 sinfo
            return _orig_makeRecord(self, name, level, fn, lno, msg, args,
                                    exc_info, func, extra)

    _logging.Logger.makeRecord = _utf8_makeRecord
    _logging.Logger._ros_utf8_patched = True


def _u8(text):
    if isinstance(text, unicode):
        return text.encode('utf-8')
    return text


def _to_unicode(text):
    if isinstance(text, bytes):
        return text.decode('utf-8', 'replace')
    if isinstance(text, unicode):
        return text
    return unicode(text)


# 模块地址常量
ADDR_RELAY = 1     # 风扇 / 电灯
ADDR_ENV = 2       # 光照 / 温度 / 湿度 / CO2
ADDR_VOICE = 3     # 语音提醒
ADDR_SAFETY = 4    # 燃气 / 漏水
ADDR_CURTAIN = 9   # 窗帘

# 模块地址 -> (数据1名称, 数据2名称, 数据3名称, 数据4名称)
ADDR_FIELDS = {
    ADDR_RELAY:   ('fan', 'light', None, None),
    ADDR_ENV:     ('illuminance', 'temperature', 'humidity', 'co2'),
    ADDR_SAFETY:  ('gas', 'water_leak', None, None),
    ADDR_CURTAIN: (None, 'curtain', None, None),
}

# 开/关取值约定（来自参考实现）
VAL_OFF = 1
VAL_ON = 2


class IotSerialBridge(object):

    def __init__(self):
        if serial is None:
            rospy.logfatal('[iot_serial] 缺少 pyserial，请执行：'
                           'sudo apt-get install python-serial')
            raise RuntimeError('python-serial missing')

        self.port = rospy.get_param('~port', '/dev/IOT_net')
        self.baudrate = int(rospy.get_param('~baudrate', 9600))
        self.poll_interval = float(rospy.get_param('~poll_interval', 0.2))
        self.read_timeout = float(rospy.get_param('~read_timeout', 0.5))

        self.lock = threading.RLock()
        self.ser = None
        self.connected = False

        self.pub_env = rospy.Publisher('/iot/serial/env', String, queue_size=5, latch=True)
        self.pub_status = rospy.Publisher('/iot/serial/status', String, queue_size=5, latch=True)

        self.sub_cmd = rospy.Subscriber('/iot/cmd', String, self.on_cmd, queue_size=10)
        self.srv_query = rospy.Service('/iot/serial/query', Trigger, self.on_query)
        self.srv_reconnect = rospy.Service('/iot/serial/reconnect', Trigger, self.on_reconnect)

        self._open()

        if self.poll_interval > 0:
            self.timer = rospy.Timer(rospy.Duration(self.poll_interval), self.on_timer)

    # ------------------------------------------------------------------ 串口
    def _open(self):
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.read_timeout)
                self.connected = True
                self._set_status(u'串口网关已连接：%s @ %d' % (self.port, self.baudrate))
            except Exception as exc:
                self.connected = False
                self._set_status(u'串口网关打开失败（%s）：%s' % (self.port, exc))

    # -------------------------------------------------------------- 协议编解码
    @staticmethod
    def build_frame(address, command, d1=0.0, d2=0.0, d3=0.0, d4=0.0):
        """构造 21 字节写帧。"""
        frame = bytearray(21)
        frame[0] = 0xA5
        frame[1] = address & 0xFF
        frame[2] = command & 0xFF
        for i, val in enumerate((d1, d2, d3, d4)):
            packed = struct.pack('<f', float(val))
            frame[3 + i * 4:7 + i * 4] = bytearray(packed)
        frame[19] = 0x0D
        frame[20] = 0x0A
        return bytes(frame)

    @staticmethod
    def parse_frame(raw):
        """解析 21 字节读帧，返回 (address, d1, d2, d3, d4) 或 None。"""
        if raw is None or len(raw) != 21:
            return None
        # 首字节读回来可能是 0xA5 的有符号表示
        if (raw[0] & 0xFF) != 0xA5:
            return None
        address = raw[1] & 0xFF
        vals = []
        for i in range(4):
            chunk = bytes(bytearray(raw[3 + i * 4:7 + i * 4]))
            try:
                vals.append(struct.unpack('<f', chunk)[0])
            except struct.error:
                vals.append(0.0)
        return (address, vals[0], vals[1], vals[2], vals[3])

    def write_module(self, address, command, d1=0.0, d2=0.0, d3=0.0, d4=0.0):
        """写一个模块，返回 (ok, message)。"""
        with self.lock:
            if self.ser is None or not self.connected:
                self._open()
            if self.ser is None:
                return False, u'串口未打开'
            try:
                frame = self.build_frame(address, command, d1, d2, d3, d4)
                self.ser.write(frame)
                self.ser.flush()
                rospy.logdebug('[iot_serial] write addr=%d cmd=%d %.2f %.2f %.2f %.2f',
                               address, command, d1, d2, d3, d4)
                return True, u'已写入地址 %d' % address
            except Exception as exc:
                self.connected = False
                return False, u'串口写入失败：%s' % repr(exc)

    def read_module(self, address, command=1, timeout=None):
        """读一个模块，返回 (parsed_tuple, message)。"""
        timeout = timeout or self.read_timeout
        deadline = rospy.get_time() + timeout
        with self.lock:
            if self.ser is None or not self.connected:
                self._open()
            if self.ser is None:
                return None, u'串口未打开'
            try:
                self.ser.write(self.build_frame(address, command, 0, 0, 0, 0))
                self.ser.flush()
                buf = bytearray()
                while rospy.get_time() < deadline and len(buf) < 21:
                    chunk = self.ser.read(21 - len(buf))
                    if chunk:
                        buf.extend(bytearray(chunk))
                parsed = self.parse_frame(buf)
                if parsed is None:
                    return None, u'读回数据不完整或帧头错误（收到 %d 字节）' % len(buf)
                return parsed, u'OK'
            except Exception as exc:
                self.connected = False
                return None, u'串口读取失败：%s' % repr(exc)

    # -------------------------------------------------------------- 语义化控制
    def set_device(self, device, value):
        """value 为 True/False 或 'on'/'off'/'open'/'close'。返回 (ok, message)。"""
        if isinstance(value, (str, bytes, unicode)):
            value = _to_unicode(value).strip().lower() in ('on', 'open', '1', 'true', 'yes')
        state = VAL_ON if value else VAL_OFF

        if device == 'fan':
            ok, msg = self.write_module(ADDR_RELAY, 1, state, 0, 0, 0)
            return ok, u'风扇已%s' % (u'开启' if value else u'关闭') if ok else msg
        if device == 'light':
            ok, msg = self.write_module(ADDR_RELAY, 1, 0, state, 0, 0)
            return ok, u'电灯已%s' % (u'开启' if value else u'关闭') if ok else msg
        if device == 'curtain':
            ok, msg = self.write_module(ADDR_CURTAIN, 1, 0, state, 0, 0)
            return ok, u'窗帘已%s' % (u'打开' if value else u'关闭') if ok else msg
        return False, u'串口通道不支持的设备：%s（支持 fan/light/curtain）' % device

    def read_env(self):
        """读取环境模块与安防模块，返回 env dict。"""
        env = {}
        parsed, msg = self.read_module(ADDR_ENV)
        if parsed:
            _addr, lux, temp, hum, co2 = parsed
            env['illuminance'] = {'value': lux, 'unit': 'lx', 'desc': u'光照强度'}
            env['temperature'] = {'value': temp, 'unit': u'°C', 'desc': u'温度'}
            env['humidity'] = {'value': hum, 'unit': '%', 'desc': u'湿度'}
            env['co2'] = {'value': co2, 'unit': 'ppm', 'desc': u'二氧化碳'}
        else:
            env['_env_error'] = msg

        parsed, msg = self.read_module(ADDR_SAFETY)
        if parsed:
            _addr, gas, leak, _a, _b = parsed
            env['gas'] = {
                'value': gas, 'desc': u'燃气',
                'text': u'正常' if gas == VAL_OFF else (u'泄漏，请及时处理' if gas == VAL_ON else u'未知'),
            }
            env['water_leak'] = {
                'value': leak, 'desc': u'漏水检测',
                'text': u'正常' if leak == VAL_OFF else (u'已经发生，请及时处理' if leak == VAL_ON else u'未知'),
                'abnormal': leak == VAL_ON,
            }
        else:
            env['_safety_error'] = msg

        return env

    # ------------------------------------------------------------------ 事件
    def on_timer(self, _evt):
        env = self.read_env()
        self.pub_env.publish(String(data=_u8(json.dumps(env, ensure_ascii=False))))

    def on_cmd(self, msg):
        raw = _to_unicode(msg.data).strip()
        if not raw:
            return
        try:
            cmd = json.loads(raw)
        except ValueError:
            self._set_status(u'指令 JSON 解析失败：%s' % raw)
            return
        device = cmd.get('device')
        action = cmd.get('action')
        if device == '__all__' and action == 'report':
            env = self.read_env()
            self._set_status(json.dumps(env, ensure_ascii=False))
            return
        ok, text = self.set_device(device, action in ('on', 'open'))
        self._set_status(text)

    def on_query(self, req):
        raw = _to_unicode(getattr(req, 'message', '') or '').strip()
        try:
            address = int(raw)
        except ValueError:
            return TriggerResponse(success=False, message=_u8(u'请给出模块地址，如 "2"'))
        parsed, msg = self.read_module(address)
        if parsed is None:
            return TriggerResponse(success=False, message=_u8(msg))
        addr, d1, d2, d3, d4 = parsed
        return TriggerResponse(success=True, message=_u8(json.dumps(
            {'address': addr, 'fields': ADDR_FIELDS.get(addr),
             'd1': d1, 'd2': d2, 'd3': d3, 'd4': d4})))

    def on_reconnect(self, _req):
        self._open()
        return TriggerResponse(success=self.connected,
                               message=_u8(u'已重连' if self.connected else u'重连失败'))

    # ------------------------------------------------------------------ 工具
    def _set_status(self, text):
        self.pub_status.publish(String(data=_u8(text)))
        rospy.loginfo('[iot_serial] %s', _u8(text))


if __name__ == '__main__':
    setup_stdout()
    fix_ros_logging()
    rospy.init_node('iot_serial_bridge')
    try:
        IotSerialBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
