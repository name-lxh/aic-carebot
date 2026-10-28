#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
sim_stubs.py — 仿真服务桩（让比赛流程能在没有硬件时走完）

## 为什么需要"桩"而不是"跳过"

`mission_manager` 的流程依赖三个外部服务：

    /qr/detect_once    二维码识别      → 返回房间巡视顺序
    /face/verify       人脸核验        → 通过/不通过
    /iot/refresh       刷新传感器      → 光照/温湿度/烟雾/漏水
    /iot/control       控制设备        → 电灯/窗帘/风扇

仿真里这些硬件都不存在。有两种做法：

    ❌ 在 mission_manager 里加 `skip_services` 直接 return True
       —— 能跑通，但**没测到服务接口对接**。
          接口名写错、消息类型不匹配、参数顺序不对，这些都不会暴露。

    ✅ 起一个**服务桩**，真的提供这些服务
       —— 测的是完整链路：mission_manager → ROS service → 桩
          接口对不对、调用时机对不对、超时处理对不对，全都测到了。

本节点就是后者。到实车时把这些桩**换成真节点**即可，
`mission_manager` 一行都不用改。

## 桩返回什么

返回**比赛场景下合理**的值，而不是固定的假值：

    /qr/detect_once    → 顺序 "2,3,4"（卧室→厨房→书房）
    /face/verify       → 通过（可参数化成失败，用来测"核验不过"的分支）
    /iot/refresh       → 光照 320lux、温 24.5℃、湿 55%、无烟雾、无漏水
    /iot/control       → 成功（可参数化成功率，用来测重试逻辑）

⚠️ 它**只做服务应答，不模拟物理**。比如"打开风扇"它只回 success，
   不会真的改变什么状态。要测状态反馈得用真的 HomeAssistant 或
   更完整的模拟器。

用法（通常由 sim_arena.launch 的 with_mission:=true 带起来）：
    rosrun carebot_eldercare sim_stubs.py
    rosrun carebot_eldercare sim_stubs.py _face_pass:=false   # 测核验不过
"""

from __future__ import print_function
from __future__ import unicode_literals

import os as _os
import sys

# ---------------------------------------------------------------------------
# 输出编码
# ---------------------------------------------------------------------------
# ⚠️ 原来说"必须用 codecs.getwriter 兜底" —— **这句是错的，已改**。
#
#     裸 `codecs.getwriter('utf-8')(stream)` 返回标准 StreamWriter，
#     它**只接受 unicode**。而 Py2 下有两类代码往流里写**字节串**：
#
#         traceback.print_exc()    → file.write(str)      字节串
#         logging.StreamHandler    → stream.write(msg+'\n')
#
#     于是它先 `decode('ascii')` → 崩：
#         UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5
#     后果是"**为了看清异常而打印 traceback 时反而崩掉**"，
#     真正的错误彻底看不到。本项目为此排查了很久。
#
# ✔ 正确做法：自定义 `codecs.StreamWriter` 子类覆写 `write()` 以容忍 bytes。
#   项目里这份实现在 `carebot_nav_kit/scripts/utf8_stdio.py`，
#   这里优先复用它（**编码处理只应有一份权威实现**）。
#
# ⚠️ 注意：这里**只能有一段**。第一版不小心写了两段重复的，
#    第二段对已经被包过的 stdout 再包一次，直接导致 UnicodeDecodeError。
#    这类"看起来无害的重复"很难查。
_HERE = _os.path.dirname(_os.path.abspath(__file__))
for _cand in (_HERE,
              _os.path.abspath(_os.path.join(_HERE, '..', '..',
                                             'carebot_nav_kit',
                                             'scripts'))):
    if _os.path.isdir(_cand) and _cand not in sys.path:
        sys.path.insert(0, _cand)

try:
    from utf8_stdio import setup_all as setup_stdout
except ImportError:
    def setup_stdout():
        import codecs

        class _Utf8Writer(codecs.StreamWriter):
            def write(self, s):
                if s is None:
                    return
                try:
                    if isinstance(s, bytes):
                        self.stream.write(s)
                        return
                    self.stream.write(s.encode('utf-8',
                                                'backslashreplace'))
                except Exception:
                    try:
                        self.stream.write(repr(s))
                    except Exception:
                        pass

        for _name in ('stdout', 'stderr'):
            _st = getattr(sys, _name, None)
            if _st is None:
                continue
            try:
                _buf = getattr(_st, 'buffer', _st)
                setattr(sys, _name,
                        _Utf8Writer(_buf, errors='backslashreplace'))
            except Exception:
                pass


setup_stdout()

import json

import rospy
from std_srvs.srv import Trigger, TriggerResponse


def fix_ros_logging():
    """修复 ROS 日志打印中文崩溃的问题（Python 2 下必现）。

    ## 背景

    ROS Melodic 跑 Python 2.7，`str(unicode)` 默认用 **ASCII** 编码。
    而 ROS 的日志处理器就是这么转的：

        rosgraph/roslogging.py:249
            msg = msg.replace('${message}', str(record_message))

    于是 `rospy.loginfo(u'中文')` 直接抛

        UnicodeEncodeError: 'ascii' codec can't encode characters

    ## 为什么这对本节点是致命的

    本节点的四个 `rospy.Service(...)` 和随后的 `loginfo` 都在 `__init__` 里。
    第一句中文日志一抛异常，**构造函数就中断了** ——
    四个服务一个都没注册上。

    表现是"服务不存在"，看起来像 launch 没起这个节点，
    **完全看不出是日志编码问题**。实测就卡在这里很久。

    ## 解法

    在 record 进入 handler **之前**把 unicode 消息转成 UTF-8 字节串。
    这样 `str(record_message)` 变成"字节串转字节串"，是无操作。

    ⚠️ 与 `nodes/mission_manager.py` 里的同名函数逻辑一致 ——
       那边在 nodes/ 下，scripts/ 里 import 不方便，所以内联一份。
       改了一处记得同步另一处。
    """
    import logging as _logging

    if getattr(_logging.Logger, '_ros_utf8_patched', False):
        return
    try:
        _text_types = (unicode,)          # noqa: F821  (Py2)
    except NameError:
        _text_types = (str,)              # Py3

    _orig = _logging.Logger.makeRecord

    def _utf8_makeRecord(self, name, level, fn, lno, msg, args, exc_info,
                         func=None, extra=None, sinfo=None):
        if isinstance(msg, _text_types):
            try:
                msg = msg.encode('utf-8')
            except Exception:
                pass
        if args:
            try:
                args = tuple(
                    a.encode('utf-8') if isinstance(a, _text_types) else a
                    for a in args)
            except Exception:
                pass
        try:
            return _orig(self, name, level, fn, lno, msg, args,
                         exc_info, func, extra, sinfo)
        except TypeError:
            return _orig(self, name, level, fn, lno, msg, args,
                         exc_info, func, extra)

    _logging.Logger.makeRecord = _utf8_makeRecord
    _logging.Logger._ros_utf8_patched = True
# ★ 控制服务必须用自定义类型 —— 见下面 on_control 的说明
from carebot_iot_msgs.srv import Control, ControlResponse


class Stubs(object):

    def __init__(self):
        # ---- 可配置 ----
        self.qr_order = rospy.get_param('~qr_order', '2,3,4')
        self.face_pass = bool(rospy.get_param('~face_pass', True))
        self.control_fail_times = int(
            rospy.get_param('~control_fail_times', 0))
        self.lux = float(rospy.get_param('~lux', 320.0))
        self.temp = float(rospy.get_param('~temperature', 24.5))
        self.humid = float(rospy.get_param('~humidity', 55.0))
        self.smoke = bool(rospy.get_param('~smoke', False))
        self.leak = bool(rospy.get_param('~leak', False))

        self.control_calls = []
        self._fail_left = self.control_fail_times

        # ---- 服务 ----
        rospy.Service('/qr/detect_once', Trigger, self.on_qr)
        rospy.Service('/face/verify', Trigger, self.on_face)
        rospy.Service('/iot/refresh', Trigger, self.on_refresh)
        # ★ 控制用 Control（自定义类型），不是 Trigger
        rospy.Service('/iot/control', Control, self.on_control)

        # ⚠️⚠️ 日志的中文必须能打出来 —— 这是**节点能不能活下来**的问题，
        #       不是"好看不好看"的问题。
        #
        # 实测踩过：本节点原来没做这个处理，结果第一句
        #     rospy.loginfo(u'[stubs] 仿真服务桩已就绪')
        # 就把节点打死了：
        #
        #     File "rosgraph/roslogging.py", line 249, in emit
        #         msg = msg.replace('${message}', str(record_message))
        #     UnicodeEncodeError: 'ascii' codec can't encode characters
        #
        # 因为 ROS Melodic 跑 Py2，`str(unicode)` 默认按 ASCII 编码。
        # 而**每个 rospy.Service 之后紧跟的 loginfo 都在 __init__ 里** ——
        # 异常一抛，四个服务全部没注册上，
        # 表现是"服务不存在"，看起来像 launch 没起节点，
        # 完全看不出是日志编码问题。
        #
        # `mission_manager.py` 里有现成的 fix_ros_logging()，这里复用。
        # （该文件在 nodes/ 下，scripts/ 下直接 import 不方便，
        #   所以把同样的补丁内联在这里，逻辑一致。）
        #
        # ⚠️ 函数名统一成 `fix_ros_logging`（去掉下划线前缀）：
        #    原来叫 `_fix_ros_logging`，**功能是对的**，但自检
        #    `test_rospy_logging_nodes_have_fix` 是按 `fix_ros_logging`
        #    这个名字找的，名字不一致会让自检**永远报警**，
        #    掩盖了真正没打补丁的节点。统一之后自检才有意义。
        fix_ros_logging()

        # ⚠️⚠️ 日志一律用 **ASCII**，不要中文。
        #
        # 这不是"图省事"，是被逼的 —— 实测反复踩：
        #
        #   ROS Melodic 跑 Py2，日志栈里有两处会做
        #   `str(unicode)`，而 Py2 的 `str()` 默认按 **ASCII** 编码：
        #
        #     1. rosgraph/roslogging.py:249
        #            msg = msg.replace('${message}', str(record_message))
        #     2. 我们自己 patch 的 makeRecord（把 unicode 转 UTF-8 字节串）
        #
        #   两处一叠加，就会出现
        #       UnicodeDecodeError: 'ascii' codec can't decode byte 0xe8
        #   —— 注意是 **Decode**Error：字节串又被当成文本解码了一次。
        #
        #   而异常一抛，`__init__` 就中断 → **四个服务一个都没注册上**。
        #   表现是"服务不存在"，看起来像 launch 没起节点，
        #   完全联想不到是"日志里有中文"。
        #
        # `fix_ros_logging()` 仍然保留 —— 它对 mission_manager 那种
        # 必须打中文的场景有用，这里当兜底。但**本节点不依赖它**。
        rospy.loginfo('[stubs] ready: 4 services registered')

    def _log(self, fmt, *args):
        """本节点专用的日志：只接受 ASCII 格式串。

        ⚠️ 为什么单独包一层：把"日志必须 ASCII"这件事**落到代码上**，
           而不是靠写代码的人每次记得。
           中文说明写在注释里，日志里只放 ASCII 事实。
        """
        rospy.loginfo('[stubs] ' + fmt, *args)

    # ---------------------------------------------------------------- 二维码
    def on_qr(self, req):
        # ⚠️ message 格式要和 qr_detector 的真实实现一致：
        #       "2,3,4|原始载荷"
        #    mission_manager 按 '|' 分割取后半段当载荷。
        #    这里保持一致，否则测不出真实接口。
        payload = 'ORDER=%s' % self.qr_order
        msg = '%s|%s' % (self.qr_order, payload)
        rospy.loginfo('[stubs] /qr/detect_once -> %s', msg)
        return TriggerResponse(success=True, message=msg)

    # ---------------------------------------------------------------- 人脸
    def on_face(self, req):
        if self.face_pass:
            rospy.loginfo('[stubs] /face/verify -> pass')
            return TriggerResponse(success=True, message='id1,0.93')
        rospy.loginfo('[stubs] /face/verify -> fail')
        return TriggerResponse(success=False, message='no face')

    # ---------------------------------------------------------------- 传感器
    def on_refresh(self, req):
        """刷新环境传感器。

        ⚠️⚠️ 返回结构必须与**真实 `/iot/refresh` 完全一致**，
              否则任务会"看起来读到了数据、但解析不出来"。

        实测踩过：第一版只返回了扁平的 env
            {"illuminance": 320.0, "temperature": 24.5, ...}
        而 `mission_manager` 期望的是**两层嵌套**（与 iot_bridge 一致）：

            {
              'states': {...},          ← 设备开关状态
              'env': {                  ← 每个传感器是一个 dict
                 'illuminance': {'value': 320.0, 'desc': '光照',
                                 'unit': 'lux', 'configured': True},
                 ...
              },
              'errors': []
            }

        于是 `mission_manager` 里
            item = env.get(key)
            if not isinstance(item, dict) or item.get('value') is None:
                continue
        全部 continue → parts 为空 → 报 `env_empty` →
        **客厅光照/温湿度这两项（共 10 分）拿不到分**。

        而日志里 `env_raw` 显示"数据是拿到了的"，很迷惑 ——
        问题不在"读不到"，在"结构不对"。

        字段名取自 `iot_bridge.py` 的 device_map。
        """
        env = {
            'illuminance': {
                'value': self.lux, 'desc': u'光照',
                'unit': 'lux', 'configured': True,
                'entity_id': 'sensor.illuminance',
            },
            'temperature': {
                'value': self.temp, 'desc': u'温度',
                'unit': u'℃', 'configured': True,
                'entity_id': 'sensor.temperature',
            },
            'humidity': {
                'value': self.humid, 'desc': u'湿度',
                'unit': '%', 'configured': True,
                'entity_id': 'sensor.humidity',
            },
            # 二元传感器用 text 而不是 value（烟雾/漏水只有正常/报警两态）
            'smoke': {
                'value': None,
                'text': u'报警' if self.smoke else u'正常',
                'desc': u'烟雾', 'configured': True,
                'abnormal': bool(self.smoke),
                'entity_id': 'binary_sensor.smoke_kitchen',
            },
            'water_leak': {
                'value': None,
                'text': u'漏水' if self.leak else u'正常',
                'desc': u'漏水', 'configured': True,
                'abnormal': bool(self.leak),
                'entity_id': 'binary_sensor.water_leak_study',
            },
        }
        payload = {
            'states': {
                'light_bedroom': 'on',
                'curtain_bedroom': 'open',
                'fan_kitchen': 'on',
            },
            'env': env,
            'errors': [],
        }
        msg = json.dumps(payload, ensure_ascii=False)
        self._log('/iot/refresh -> ok, %d env keys', len(env))
        return TriggerResponse(success=True, message=msg)

    # ---------------------------------------------------------------- 控制
    def on_control(self, req):
        """设备控制桩。

        ⚠️ 注意这不是 `Trigger` —— Trigger 的请求是空的，
           拿不到 device/action。真实 iot_bridge 踩过这个坑
           （设备控制从未生效，比赛丢 20 分）。
           这里用 `carebot_iot_msgs/Control`，并且**把收到的内容打出来** ——
           这样"接口对不对"一眼可见，而不是"调了没反应"。
        """
        device = getattr(req, 'device', '') or ''
        action = getattr(req, 'action', '') or ''
        self.control_calls.append((device, action))
        rospy.loginfo('[stubs] /iot/control -> device=%s action=%s',
                      device, action)

        if not device or not action:
            rospy.logerr('[stubs] request missing device/action -- '
                         'service type mismatch? (Trigger instead of Control?)')
            return ControlResponse(success=False,
                                   message='missing device/action')

        if self._fail_left > 0:
            self._fail_left -= 1
            rospy.logwarn('[stubs] forced failure, %d left', self._fail_left)
            return ControlResponse(success=False,
                                   message='stub forced fail')
        return ControlResponse(success=True, message='ok')


def main():
    rospy.init_node('sim_stubs', anonymous=False)
    Stubs()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
