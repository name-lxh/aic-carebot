#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
iot_bridge.py — 智慧养老护理赛题：HomeAssistant 物联网桥接节点

为什么重写参考实现
------------------
组委会参考包 `mijia_control/src/home_assistant.py` 只覆盖了「灯 + 通用开关 + 温湿度 + 门窗」，
且存在 4 个必须修掉的缺陷：

  1. **覆盖度不足**：缺少风扇、窗帘、光照强度、烟雾/CO₂、漏水 —— 共 30 分未覆盖。
  2. **令牌硬编码在源码里**（明文 Bearer token），过期即全线失效，且是安全隐患。
  3. **用字符串切片解析 JSON**（`response.text[i:i+14] == '"temperature":'`），
     HA 返回字段顺序或数值位数一变就解析失败。
  4. **把 Subscriber 回调当循环用**：回调里写了 `while not rospy.is_shutdown()`，
     会在 ROS 回调线程里永久占死该线程。

本节点的设计
------------
  * **设备映射配置化**：所有 entity_id 放在 `device_map.yaml`，现场改配置不改代码。
  * **令牌可注入**：优先级 环境变量 HA_TOKEN > rosparam ~token > 配置文件；
    支持 `token_file`（推荐：把令牌写进只读文件，不进 git）。
  * **JSON 解析**：用 `json.loads()`，并用 HA 的 `attributes.unit_of_measurement`
    与 `state_class` 做健壮的数值提取。
  * **Timer 周期轮询 + Service 按需查询**：不在回调里阻塞。
  * **统一指令接口**：`/iot/cmd` 收 JSON，屏蔽 light/switch/cover/fan 的差异。
  * **写后回读校验**：设备控制后回读状态，未生效自动重试 —— 这是拿满 30 分的关键。
  * **双通道冗余**：HA 不可达时可选降级到串口网关（见 iot_serial_bridge.py）。

订阅
----
  /iot/cmd     std_msgs/String   JSON 指令，形如：
                                 {"device":"light_bedroom","action":"on"}
                                 {"device":"curtain_bedroom","action":"open"}
                                 {"device":"fan_kitchen","action":"on"}
                                 {"device":"sensor_power","action":"on"}
                                 {"device":"__all__","action":"report"}   # 播报全部状态

发布
----
  /iot/state   std_msgs/String   全部受控设备 + 传感器的 JSON 快照
  /iot/env     std_msgs/String   环境数据 JSON（光照/温度/湿度）
  /iot/status  std_msgs/String   人类可读状态（供语音播报与 2 号机位展示）

服务
----
  /iot/refresh   std_srvs/Trigger   立即刷新全部状态并阻塞返回
  /iot/control   std_srvs/Trigger   请求 message 为 JSON 指令，同步执行并返回是否成功

参数
----
  ~base_url      默认 http://192.168.1.240:8123
  ~token         长期访问令牌（建议改用 token_file 或环境变量 HA_TOKEN）
  ~token_file    存放令牌的文件路径（整文件内容即为令牌）
  ~poll_period   周期轮询间隔（秒），默认 2.0
  ~timeout       HTTP 超时（秒），默认 3.0
  ~verify_after  控制后是否回读校验，默认 True
  ~retry_times   回读失败重试次数，默认 2
  ~device_map    设备映射 yaml 路径（由 launch 传入）
"""

import sys
import json
import os
import threading

import rospy
import requests
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerResponse
# ★ 设备控制用**自定义服务类型**，不能用 std_srvs/Trigger。
#
# ⚠️ 这是个踩过的坑，而且丢分很重：
#
#   原来写的是 `rospy.Service('/iot/control', Trigger, self.on_control)`，
#   而 **Trigger 的请求是空的**：
#       $ cat /opt/ros/melodic/share/std_srvs/srv/Trigger.srv
#       ---
#       bool success
#       string message
#     `TriggerRequest.__slots__ == []` —— 请求里一个字段都没有。
#
#   于是两边都拿不到参数：
#       调用方  proxy(payload)              → payload 被静默丢弃
#       服务端  getattr(req,'message','')   → 永远空字符串
#   结果 JSON 解析失败、返回 success=False，
#   **"打开卧室电灯"这类指令从未生效过** —— 比赛里就是 20 分（电灯/窗帘/风扇）。
#
#   症状很隐蔽：服务能起来、服务名和类型名都对、编译也不报错，
#   只有真正调用一次、并且服务端去读请求内容才会暴露。
from carebot_iot_msgs.srv import Control, ControlResponse

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
    """统一转 UTF-8 字节串：ROS std_msgs/String 在 Py2 下不接受 unicode。"""
    if isinstance(text, unicode):
        return text.encode('utf-8')
    return text


def _to_unicode(text):
    if isinstance(text, bytes):
        return text.decode('utf-8', 'replace')
    if isinstance(text, unicode):
        return text
    return unicode(text)


def ha_api_ok(text):
    """判断 HA `/api/` 返回体是否表示服务正常。

    兼容两种真实响应（2026-10-20 在实验室 HA 上实测）：

      * 旧版 HA：纯文本 ``API running.``
      * 新版 HA（2025.x）：JSON ``{"message":"API running."}``

    之前只认纯文本，新版盒子会误报「响应异常」，但读传感器其实是通的。
    """
    body = (text or '').strip()
    if body == 'API running.':
        return True
    if body.startswith('{'):
        try:
            data = json.loads(body)
        except ValueError:
            return False
        return isinstance(data, dict) and data.get('message') == 'API running.'
    return False


class IotBridge(object):

    # 动作 -> HA 服务名。不同设备域的服务名不同，这里统一成动作语义。
    ACTION_SERVICE = {
        'on':    ('turn_on', True),
        'off':   ('turn_off', False),
        'open':  ('open_cover', True),
        'close': ('close_cover', False),
        'stop':  ('stop_cover', None),
        'toggle': ('toggle', None),
    }

    def __init__(self):
        self.base_url = rospy.get_param('~base_url', 'http://192.168.1.240:8123').rstrip('/')
        self.timeout = float(rospy.get_param('~timeout', 3.0))
        self.poll_period = float(rospy.get_param('~poll_period', 2.0))
        self.verify_after = bool(rospy.get_param('~verify_after', True))
        self.retry_times = int(rospy.get_param('~retry_times', 2))
        # ★ 回读校验窗口：Zigbee 灯/插座的状态上报有时要好几秒才回来，
        #   原来固定 5×0.4=2 s 太短，会误判"开灯失败"（2026-10-21 现场）。
        #   现在可配，且允许在 device_map.yaml 里按设备覆盖（例如灯给 12 次×0.5=6 s）。
        self.verify_attempts = int(rospy.get_param('~verify_attempts', 5))
        self.verify_interval = float(rospy.get_param('~verify_interval', 0.4))

        self.token = self._resolve_token()

        # 设备映射
        map_path = rospy.get_param('~device_map', '')
        self.device_map = self._load_device_map(map_path)

        # 扁平化：逻辑名 -> 规格
        self.controllable = dict(self.device_map.get('controllable', {}))
        self.sensors = dict(self.device_map.get('sensors', {}))
        self.groups = dict(self.device_map.get('groups', {}))

        # 缓存
        self.lock = threading.RLock()
        self.state_cache = {}       # entity_id -> state 字符串
        self.attr_cache = {}        # entity_id -> attributes dict
        self.last_ok = False

        self.session = requests.Session()
        self.session.headers.update({
            'Authorization': 'Bearer %s' % self.token,
            'Content-Type': 'application/json',
        })

        # 发布者
        self.pub_state = rospy.Publisher('/iot/state', String, queue_size=5, latch=True)
        self.pub_env = rospy.Publisher('/iot/env', String, queue_size=5, latch=True)
        self.pub_status = rospy.Publisher('/iot/status', String, queue_size=5, latch=True)

        # 订阅者 / 服务
        self.sub_cmd = rospy.Subscriber('/iot/cmd', String, self.on_cmd, queue_size=10)
        self.srv_refresh = rospy.Service('/iot/refresh', Trigger, self.on_refresh)
        # ★ 控制用自定义类型 Control（见文件头 import 处的说明）
        self.srv_control = rospy.Service('/iot/control', Control, self.on_control)

        # 兼容组委会参考平台的旧话题协议（Int32, data<=5 开 / >=6 关）
        self.legacy_subs = []
        legacy = self.device_map.get('legacy_topics', {}) or {}
        if legacy.get('enabled'):
            self.on_max = int(legacy.get('on_max', 5))
            self.off_min = int(legacy.get('off_min', 6))
            for topic, device in legacy.items():
                if topic in ('enabled', 'on_max', 'off_min'):
                    continue
                if device not in self.controllable:
                    rospy.logwarn('[iot_bridge] 旧话题 %s 指向未知设备 %s，已跳过', topic, device)
                    continue
                sub = rospy.Subscriber('/' + topic.lstrip('/'), Int32,
                                       self._make_legacy_cb(device), queue_size=10)
                self.legacy_subs.append(sub)
                rospy.loginfo('[iot_bridge] 兼容旧话题 /%s -> %s', topic.lstrip('/'), device)

        self.timer = rospy.Timer(rospy.Duration(self.poll_period), self.on_timer)

        self._set_status(u'物联网桥接就绪：%s（可控设备 %d 个，传感器 %d 个）'
                         % (self.base_url, len(self.controllable), len(self.sensors)))
        self._self_check()

    # ------------------------------------------------------------------ 初始化
    def _resolve_token(self):
        # 优先级：环境变量 > rosparam > 令牌文件
        token = os.environ.get('HA_TOKEN', '').strip()
        if token:
            rospy.loginfo('[iot_bridge] 令牌来源：环境变量 HA_TOKEN')
            return token

        token = str(rospy.get_param('~token', '')).strip()
        if token:
            rospy.loginfo('[iot_bridge] 令牌来源：rosparam ~token')
            return token

        token_file = str(rospy.get_param('~token_file', '')).strip()
        if token_file and os.path.isfile(token_file):
            with open(token_file, 'r') as fp:
                token = fp.read().strip()
            if token:
                rospy.loginfo('[iot_bridge] 令牌来源：文件 %s', token_file)
                return token

        rospy.logwarn('[iot_bridge] 未配置长期访问令牌！'
                      '请设置环境变量 HA_TOKEN、rosparam ~token 或 ~token_file。')
        return ''

    @staticmethod
    def _load_device_map(path):
        """读取设备映射 yaml；路径为空或文件不存在时使用内置兜底映射。"""
        if path and os.path.isfile(path):
            try:
                import yaml
                with open(path, "rb") as fp:
                    data = yaml.safe_load(fp.read()) or {}
                rospy.loginfo('[iot_bridge] 已加载设备映射：%s', path)
                return data
            except Exception as exc:
                rospy.logerr('[iot_bridge] 设备映射加载失败 %s: %s', path, exc)

        rospy.logwarn('[iot_bridge] 使用内置兜底设备映射（建议改用 device_map.yaml）')
        return {
            'controllable': {
                'light_bedroom':   {'entity_id': 'light.0x54ef441000707d76',
                                    'domain': 'light', 'desc': u'卧室电灯'},
                'curtain_bedroom': {'entity_id': 'cover.curtain_bedroom',
                                    'domain': 'cover', 'desc': u'卧室窗帘',
                                    'on_state': 'open', 'off_state': 'closed'},
                'fan_kitchen':     {'entity_id': 'switch.0xa4c1384d3c4d8403',
                                    'domain': 'switch', 'desc': u'厨房风扇'},
                'sensor_power':    {'entity_id': 'switch.sensor_power',
                                    'domain': 'switch', 'desc': u'厨房传感器电源'},
            },
            'sensors': {
                'illuminance':  {'entity_id': 'sensor.illuminance',
                                 'desc': u'光照强度', 'unit': 'lx'},
                'temperature':  {'entity_id': 'sensor.0x00124b00252be679_temperature',
                                 'desc': u'温度', 'unit': u'°C'},
                'humidity':     {'entity_id': 'sensor.0x00124b00252be679_humidity',
                                 'desc': u'湿度', 'unit': '%'},
                'smoke':        {'entity_id': 'binary_sensor.smoke_kitchen',
                                 'desc': u'烟雾报警', 'on_state': 'on',
                                 'on_text': u'检测到烟雾', 'off_text': u'正常'},
                'water_leak':   {'entity_id': 'binary_sensor.water_leak_study',
                                 'desc': u'漏水检测', 'on_state': 'on',
                                 'on_text': u'发生漏水', 'off_text': u'正常'},
            },
            'groups': {},
        }

    def _self_check(self):
        """启动自检：确认 HA 可达，并把当前设备清单打印到日志。"""
        try:
            r = self.session.get('%s/api/' % self.base_url, timeout=self.timeout)
            if r.status_code == 200:
                body = r.text.strip()
                if ha_api_ok(body):
                    self._set_status(u'HomeAssistant 连接正常：%s' % self.base_url)
                    self.last_ok = True
                    return
                self._set_status(u'HomeAssistant 响应异常：%s' % body[:80])
            elif r.status_code == 401:
                self._set_status(u'HomeAssistant 令牌无效（401），请更新长期访问令牌')
            else:
                self._set_status(u'HomeAssistant HTTP %d' % r.status_code)
        except Exception as exc:
            self._set_status(u'HomeAssistant 不可达（%s）：%s' % (self.base_url, exc))

    # ------------------------------------------------------------------ 基础 HTTP
    def _get_json(self, path):
        url = '%s%s' % (self.base_url, path)
        r = self.session.get(url, timeout=self.timeout)
        if r.status_code != 200:
            raise IOError('GET %s -> HTTP %d' % (url, r.status_code))
        return json.loads(r.text)

    def _post_service(self, domain, service, entity_id, extra=None):
        url = '%s/api/services/%s/%s' % (self.base_url, domain, service)
        payload = {'entity_id': entity_id}
        if extra:
            payload.update(extra)
        r = self.session.post(url, data=json.dumps(payload), timeout=self.timeout)
        if r.status_code not in (200, 201):
            raise IOError('POST %s -> HTTP %d: %s' % (url, r.status_code, r.text[:120]))
        return r

    # -------------------------------------------------------------- 状态读取
    def _read_state(self, entity_id):
        data = self._get_json('/api/states/%s' % entity_id)
        state = data.get('state')
        attrs = data.get('attributes') or {}
        with self.lock:
            self.state_cache[entity_id] = state
            self.attr_cache[entity_id] = attrs
        return state, attrs

    @staticmethod
    def _numeric(state, attrs):
        """从 HA 状态里稳健地取出数值。

        优先用 attributes 里的数值字段，退化到解析 state 字符串
        （HA 对某些传感器会把数值放在 state 里，如 "23.5"）。
        """
        for key in ('value', 'state', 'temperature', 'humidity', 'illuminance'):
            v = attrs.get(key)
            if isinstance(v, (int, float)):
                return float(v)
        if state is None:
            return None
        text = _to_unicode(state).strip()
        try:
            return float(text)
        except ValueError:
            pass
        # 退化：抓第一个数字
        import re as _re
        m = _re.search(r'-?\d+(\.\d+)?', text)
        if m:
            try:
                return float(m.group(0))
            except ValueError:
                return None
        return None

    def refresh_all(self):
        """刷新全部设备与传感器状态，返回 (state_dict, env_dict, errors)。"""
        errors = []
        states = {}
        env = {}

        for name, spec in self.controllable.items():
            eid = spec.get('entity_id')
            if not eid:
                continue
            try:
                s, _a = self._read_state(eid)
                states[name] = s
            except Exception as exc:
                errors.append('%s: %s' % (name, exc))

        for name, spec in self.sensors.items():
            eid = spec.get('entity_id')
            if not eid:
                # 现场可能尚未确认该传感器的 entity_id（如漏水），跳过而不是报错
                env[name] = {'text': spec.get('off_text', u'未配置传感器'), 'raw': None,
                             'desc': spec.get('desc', name), 'configured': False}
                continue
            try:
                s, a = self._read_state(eid)
                states[name] = s
                val = self._numeric(s, a)
                # ★ 2026-10-24 现场：HA 里温度实体是 °F，按配置换算成 °C
                if val is not None and spec.get('convert') == 'f_to_c':
                    val = round((float(val) - 32.0) / 1.8, 1)

                if 'threshold' in spec:
                    # 阈值型传感器（如烟雾浓度）：数值超过阈值即判定为异常
                    if val is None:
                        env[name] = {'text': spec.get('normal_text', u'未知'), 'raw': s,
                                     'desc': spec.get('desc', name)}
                    else:
                        abnormal = val > float(spec['threshold'])
                        env[name] = {
                            'value': val,
                            'unit': spec.get('unit') or a.get('unit_of_measurement') or '',
                            'desc': spec.get('desc', name),
                            'abnormal': abnormal,
                            'text': spec.get('threshold_text') if abnormal
                                    else spec.get('normal_text', u'正常'),
                        }
                elif val is not None:
                    env[name] = {
                        'value': val,
                        'unit': spec.get('unit') or a.get('unit_of_measurement') or '',
                        'desc': spec.get('desc', name),
                    }
                else:
                    # 二元传感器（water_leak / 报警器）
                    env[name] = {
                        'raw': s,
                        'desc': spec.get('desc', name),
                        'abnormal': s == spec.get('on_state'),
                        'text': spec.get('on_text') if s == spec.get('on_state')
                                else spec.get('off_text', _to_unicode(s)),
                    }
            except Exception as exc:
                errors.append('%s: %s' % (name, exc))

        return states, env, errors

    # -------------------------------------------------------------- 设备控制
    def control(self, device, action, extra=None):
        """控制一个逻辑设备，返回 (ok, message)。含写后回读校验与重试。"""
        spec = self.controllable.get(device)
        if spec is None:
            return False, u'未知设备：%s（可用：%s）' % (device, ', '.join(sorted(self.controllable)))

        eid = spec.get('entity_id')
        # ★ 2026-10-22：现场确实没有对应设备时映射留空，直接返回“未配置”，
        #   否则会 POST entity_id="" -> HA 400，日志里看着像真的报错。
        if not eid:
            return False, u'%s 未配置（现场无此设备，跳过）' % device
        domain = spec.get('domain') or eid.split('.')[0]
        if action not in self.ACTION_SERVICE:
            return False, u'未知动作：%s（可用：%s）' % (action, ', '.join(sorted(self.ACTION_SERVICE)))
        service, expect_true = self.ACTION_SERVICE[action]

        # cover 域没有 turn_on/turn_off，做一次映射
        if domain == 'cover' and service in ('turn_on', 'turn_off'):
            service = 'open_cover' if service == 'turn_on' else 'close_cover'
        if domain in ('light', 'switch', 'fan') and service in ('open_cover', 'close_cover'):
            service = 'turn_on' if service == 'open_cover' else 'turn_off'

        expect_state = None
        if expect_true is True:
            expect_state = spec.get('on_state', 'on')
        elif expect_true is False:
            expect_state = spec.get('off_state', 'off')

        last_err = None
        for attempt in range(1, self.retry_times + 2):
            try:
                self._post_service(domain, service, eid, extra)
            except Exception as exc:
                last_err = exc
                rospy.logwarn('[iot_bridge] %s/%s 第 %d 次调用失败: %s',
                              device, action, attempt, exc)
                rospy.sleep(0.3)
                continue

            if not self.verify_after or expect_state is None:
                self._set_status(u'已下发 %s -> %s' % (spec.get('desc', device), action))
                return True, u'%s %s 已下发' % (device, action)

            # 回读校验（设备响应有延迟，给几次机会；次数/间隔可按设备覆盖）
            n_att = int(spec.get('verify_attempts', self.verify_attempts))
            interval = float(spec.get('verify_interval', self.verify_interval))
            for _ in range(max(1, n_att)):
                rospy.sleep(interval)
                try:
                    s, _a = self._read_state(eid)
                except Exception as exc:
                    last_err = exc
                    continue
                if s == expect_state:
                    self._set_status(u'%s 已%s（回读 state=%s）'
                                     % (spec.get('desc', device), action, s))
                    return True, u'%s %s 成功' % (device, action)

            rospy.logwarn('[iot_bridge] %s 回读未达预期 state=%s（期望 %s），第 %d 次重试',
                          device, self.state_cache.get(eid), expect_state, attempt)

        return False, u'%s %s 失败（已重试 %d 次）：%s' % (
            device, action, self.retry_times + 1, last_err)

    # ------------------------------------------------------------------ 播报文本
    def describe_env(self, keys=None):
        """把环境数据组织成一句中文播报文本。

        keys 为 None 时播报全部；也可只播报指定项，例如
        describe_env(['illuminance', 'temperature', 'humidity']) 用于客厅。
        """
        try:
            _s, env, _e = self.refresh_all()
        except Exception as exc:
            return u'环境数据读取失败：%s' % repr(exc)

        if keys is None:
            keys = ('illuminance', 'temperature', 'humidity', 'smoke', 'water_leak')

        parts = []
        for key in keys:
            item = env.get(key)
            if not item:
                continue
            desc = item.get('desc', key)
            if item.get('configured') is False:
                continue
            if 'value' in item and item['value'] is not None:
                unit = item.get('unit') or ''
                suffix = u'，%s' % item['text'] if item.get('abnormal') else u''
                parts.append(u'%s %.1f%s%s' % (desc, item['value'], unit, suffix))
            elif 'text' in item:
                parts.append(u'%s%s' % (desc, item['text']))

        if not parts:
            return u'未能读取到环境数据'
        return u'当前环境：' + u'，'.join(parts)

    # ------------------------------------------------------------------ 事件
    def _make_legacy_cb(self, device):
        """生成旧话题协议的回调（Int32：<=on_max 开，>=off_min 关）。"""
        def _cb(msg):
            action = 'on' if msg.data <= self.on_max else 'off'
            ok, text = self.control(device, action)
            if not ok:
                self._set_status(text)
        return _cb

    def on_timer(self, _evt):
        if self.poll_period <= 0:
            return
        try:
            states, env, errors = self.refresh_all()
            self.last_ok = not errors
            self.pub_state.publish(String(data=_u8(json.dumps(
                {'states': states, 'errors': errors}, ensure_ascii=False))))
            self.pub_env.publish(String(data=_u8(json.dumps(env, ensure_ascii=False))))
        except Exception as exc:
            rospy.logwarn_throttle(10.0, '[iot_bridge] 周期轮询失败: %s', exc)

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
            self._set_status(self.describe_env())
            return

        # 支持一条指令控制一组设备
        if device in self.groups:
            members = self.groups[device].get('members', [])
            ok_all = True
            for m in members:
                ok, msgtext = self.control(m, action, cmd.get('extra'))
                ok_all = ok_all and ok
                self._set_status(msgtext)
            return

        ok, msgtext = self.control(device, action, cmd.get('extra'))
        if not ok:
            self._set_status(msgtext)

    def on_refresh(self, _req):
        try:
            states, env, errors = self.refresh_all()
            self.pub_state.publish(String(data=_u8(json.dumps(
                {'states': states, 'errors': errors}, ensure_ascii=False))))
            self.pub_env.publish(String(data=_u8(json.dumps(env, ensure_ascii=False))))
            return TriggerResponse(
                success=not errors,
                message=_u8(json.dumps({'states': states, 'env': env, 'errors': errors},
                                       ensure_ascii=False)))
        except Exception as exc:
            return TriggerResponse(success=False, message=_u8(u'刷新失败：%s' % repr(exc)))

    def on_control(self, req):
        """处理设备控制请求。

        ⚠️ 入参是 `carebot_iot_msgs/Control` 的**请求**，字段名是
           `device` / `action` / `extra`，直接读即可。

           原来是 `getattr(req, 'message', '')` 去读一个不存在的字段
           （因为服务类型是 std_srvs/Trigger，请求是空的）——
           那行代码**永远拿到空字符串**，设备控制从未生效。
           见文件头 import 处的详细说明。
        """
        device = _to_unicode(getattr(req, 'device', '') or '').strip()
        action = _to_unicode(getattr(req, 'action', '') or '').strip()
        extra_raw = _to_unicode(getattr(req, 'extra', '') or '').strip()

        if not device or not action:
            return ControlResponse(
                success=False,
                # ⚠️ 不能用 %r：%r 产出**字节串**，插进 unicode 格式串时
                #    Py2 会按 ASCII 解码它 —— device/action 若含非 ASCII
                #    就崩。而这行正在"参数不对"的错误分支里，
                #    崩了就看不到到底收到了什么。
                message=_u8(u'缺少 device 或 action（收到 device=%s action=%s）'
                            % (_to_unicode(device), _to_unicode(action))))

        extra = None
        if extra_raw:
            try:
                extra = json.loads(extra_raw)
            except ValueError:
                return ControlResponse(
                    success=False,
                    message=_u8(u'extra 不是合法 JSON：%s' % extra_raw))

        ok, msgtext = self.control(device, action, extra)
        return ControlResponse(success=ok, message=_u8(msgtext))

    # ------------------------------------------------------------------ 工具
    def _set_status(self, text):
        self.pub_status.publish(String(data=_u8(text)))
        rospy.loginfo('[iot_bridge] %s', _u8(text))


if __name__ == '__main__':
    setup_stdout()
    fix_ros_logging()
    rospy.init_node('iot_bridge')
    try:
        IotBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
