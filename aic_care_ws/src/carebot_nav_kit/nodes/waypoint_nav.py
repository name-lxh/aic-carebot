#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
waypoint_nav.py — 航点导航节点

用途
----
把"导航到某个点"封装成一个带**超时、重试、降级**的可靠动作，并把它暴露成
服务与话题接口，供上层任务状态机调用。

为什么需要它（而不是直接用 move_base_simple/goal）
--------------------------------------------------
`rostopic pub /move_base_simple/goal` 只是"发一个目标"，发完就不管了：
  · 不知道到没到
  · 失败了不会重试
  · 卡住了不会超时
  · 失败了上层无从得知

赛题规则规定"1 分钟内未对项目情景做出回应"即判故障，再犯出局。
所以每个导航动作都必须有明确的超时与结果反馈 —— 本节点提供这一层。

接口
----
服务：
  /nav/goto            std_srvs/Trigger    请求 message 格式：
                                           "名称"          导航到已配置航点
                                           "x,y,yaw"       导航到任意位姿
                       message 返回 JSON：{"ok":bool,"name":..,"attempts":..,
                                           "elapsed":..,"message":..}

  /nav/goto_named      (同上，语义更明确的别名)
  /nav/capture_pose    std_srvs/Trigger    记录当前位姿到指定名字（用于标定航点）
                                           请求 message = "名字"
  /nav/cancel          std_srvs/Trigger    取消当前导航
  /nav/reload          std_srvs/Trigger    重新加载航点文件
  /nav/status          std_srvs/Trigger    返回当前状态 JSON

话题：
  /nav/status          std_msgs/String     状态变化（人类可读，供界面/语音）
  /nav/result          std_msgs/String     每次导航结果 JSON
  /nav/current_index   std_msgs/Int32      当前航点序号（按序列导航时）

参数
----
  ~waypoints_file   航点 YAML 路径
  ~goal_timeout     单点导航超时（秒），默认 60
  ~max_retries      失败重试次数，默认 2
  ~retry_delay      重试间隔（秒），默认 0.5
  ~frame_id         目标坐标系，默认 map
  ~arrive_settle    到达后稳定时间（秒），默认 0.5

航点文件格式
------------
  frame_id: map
  waypoints:
    point_A: {x: 1.0, y: -0.8, yaw: 1.5708}
    room_B:  {x: 2.2, y: 1.2,  yaw: 0.0}
"""

import io
import json
import math
import os
import sys
import threading
import time

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
    """统一转 UTF-8 字节串（Py2 下 ROS String 不接受 unicode）。"""
    if isinstance(text, unicode):
        return text.encode('utf-8')
    return text


def _to_unicode(text):
    if isinstance(text, bytes):
        return text.decode('utf-8', 'replace')
    if isinstance(text, unicode):
        return text
    return unicode(text)


class WaypointNav(object):

    def __init__(self):
        import rospy
        import actionlib
        from move_base_msgs.msg import MoveBaseAction

        self.rospy = rospy
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 60.0))
        self.max_retries = int(rospy.get_param('~max_retries', 2))
        self.retry_delay = float(rospy.get_param('~retry_delay', 0.5))
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.arrive_settle = float(rospy.get_param('~arrive_settle', 0.5))
        self.waypoints_file = rospy.get_param('~waypoints_file', '')

        self.lock = threading.RLock()
        self.busy = False
        self.cancelled = False
        self.waypoints = {}
        self.waypoint_order = []

        self._load_waypoints(self.waypoints_file)

        # --- move_base 动作客户端 ---
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.server_ready = False

        # --- 发布者 ---
        from std_msgs.msg import String, Int32
        self.pub_status = rospy.Publisher('/nav/status', String, queue_size=10, latch=True)
        self.pub_result = rospy.Publisher('/nav/result', String, queue_size=10)
        self.pub_index = rospy.Publisher('/nav/current_index', Int32, queue_size=5, latch=True)

        # --- 服务 ---
        from std_srvs.srv import Trigger
        self.srv_goto = rospy.Service('/nav/goto', Trigger, self.on_goto)
        self.srv_goto_named = rospy.Service('/nav/goto_named', Trigger, self.on_goto)
        self.srv_capture = rospy.Service('/nav/capture_pose', Trigger, self.on_capture)
        self.srv_cancel = rospy.Service('/nav/cancel', Trigger, self.on_cancel)
        self.srv_reload = rospy.Service('/nav/reload', Trigger, self.on_reload)
        self.srv_status = rospy.Service('/nav/status_srv', Trigger, self.on_status)

        # 后台等待 move_base（不阻塞服务注册）
        self.wait_thread = threading.Thread(target=self._wait_for_server)
        self.wait_thread.daemon = True
        self.wait_thread.start()

        self._set_status(u'航点导航就绪，已加载 %d 个航点：%s'
                         % (len(self.waypoints),
                            u', '.join(self.waypoint_order) if self.waypoint_order else u'（无）'))

    # ------------------------------------------------------------------ 初始化
    def _load_waypoints(self, path):
        if not path or not os.path.isfile(path):
            self.rospy.logwarn('[waypoint_nav] 航点文件不存在：%s' % path)
            return
        try:
            import yaml
            # ⚠️⚠️ 必须**读成字节串**再交给 yaml.safe_load，
            #       不能用 `io.open(path, encoding='utf-8')` 传文本流。
            #
            # 实测踩过（而且是**反复**踩，一直被我当成噪音漏掉）：
            #
            #     [ERROR] [waypoint_nav] 航点文件解析失败 ...：
            #             'ascii' codec can't decode byte 0xe5
            #             in position 15: ordinal not in range(128)
            #
            # 后果是**致命的、但症状很误导**：
            #     · `self.waypoints` 变成空字典
            #     · 导航时所有命名点位都取不到 → 任务一个目标点都发不出去
            #     · 日志里只剩 move_base 的 "Failed to get a plan" 刷屏
            #       —— 看起来像"规划器坏了"，其实是**点位根本没加载**
            #
            # 为什么文本流会崩：
            #     PyYAML 收到 unicode 流时，内部仍会按它自己的规则处理
            #     非 ASCII 字节；而 yaml 文件里有中文注释（0xe5 = "的" 的首字节）。
            #     喂字节串则由 PyYAML 自己按 UTF-8 解码，这条路径是稳的。
            #
            # 这个坑在 `remote_verify.py` / `scan_imports.py` 里也踩过 ——
            # **Py2 下凡是"读文件再解析"，一律读字节串。**
            with open(path, 'rb') as fp:
                data = yaml.safe_load(fp.read()) or {}
            wps = data.get('waypoints') or {}
            self.waypoints = {}
            self.waypoint_order = []
            for name in wps:
                w = wps[name]
                self.waypoints[_to_unicode(name)] = (
                    float(w.get('x', 0.0)),
                    float(w.get('y', 0.0)),
                    float(w.get('yaw', 0.0)),
                )
                self.waypoint_order.append(_to_unicode(name))
            if data.get('frame_id'):
                self.frame_id = data['frame_id']
            # ⚠️⚠️ 这句日志必须是 ASCII（或经过安全包装），否则会**伪装成解析失败**。
            #
            # 实测踩过、而且被误导了好几轮：
            #
            #   `self.rospy.loginfo(u'已加载 %d 个航点…')` 里带中文，
            #   而 Py2 下 ROS 的日志处理器会 `str(unicode)` → ASCII 编码 → 抛异常。
            #   **而这句 loginfo 就在 try 块里**，于是：
            #
            #       except Exception as exc:
            #           logerr('航点文件解析失败 %s：%s' % (path, exc))
            #
            #   结果日志里只有
            #       [ERROR] 航点文件解析失败 ...: 'ascii' codec can't decode byte 0xe5
            #   紧接着又出现
            #       [INFO]  已加载 6 个航点：qr_area, room_B, ...
            #   —— 因为**日志本身是 buffered 的，顺序错位**，
            #      看起来像"失败了一次又成功了一次"。
            #
            #   实际情况是：**YAML 解析完全成功，点位也全部加载了**，
            #   只有那句成功日志打不出来。我据此查了很久"为什么解析失败"。
            #
            # 教训：**try 块里不要放会因编码崩的日志** ——
            #       它会把成功伪装成失败，而且报的是**日志的异常**，
            #       不是真正出问题的地方。
            self.rospy.loginfo(
                '[waypoint_nav] loaded %d waypoints: %s'
                % (len(self.waypoints),
                   ', '.join(self.waypoint_order)))
        except Exception as exc:
            self.rospy.logerr('[waypoint_nav] 航点文件解析失败 %s：%s' % (path, exc))

    def _wait_for_server(self):
        self.rospy.loginfo('[waypoint_nav] 等待 move_base 动作服务器…')
        self.server_ready = self.client.wait_for_server(self.rospy.Duration(60.0))
        if self.server_ready:
            self._set_status(u'move_base 已连接，可以开始导航')
        else:
            self._set_status(u'警告：60 秒内未连接到 move_base，导航服务会返回失败')

    # ------------------------------------------------------------------ 工具
    def _set_status(self, text):
        from std_msgs.msg import String
        self.pub_status.publish(String(data=_u8(text)))
        self.rospy.loginfo('[waypoint_nav] %s' % _u8(text))

    def _current_pose(self):
        """从 TF 读取 map → base_footprint 的当前位姿。"""
        try:
            import tf
            listener = tf.TransformListener()
            self.rospy.sleep(0.5)
            listener.waitForTransform('/' + self.frame_id, '/base_footprint',
                                      self.rospy.Time(0), self.rospy.Duration(3.0))
            trans, rot = listener.lookupTransform('/' + self.frame_id, '/base_footprint',
                                                  self.rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            return trans[0], trans[1], euler[2]
        except Exception as exc:
            self.rospy.logwarn('[waypoint_nav] 读取当前位姿失败：%s' % repr(exc))
            return None

    @staticmethod
    def _parse_target(text):
        """解析目标：'名字' 或 'x,y,yaw' 或 'x,y'。返回 (kind, name_or_tuple)。"""
        text = _to_unicode(text).strip()
        if not text:
            return None, u'请求为空'
        # 尝试解析为坐标
        if ',' in text or u'，' in text:
            parts = text.replace(u'，', ',').split(',')
            try:
                vals = [float(p.strip()) for p in parts if p.strip()]
            except ValueError:
                return None, u'坐标格式错误：%s' % text
            if len(vals) == 2:
                return ('pose', (vals[0], vals[1], 0.0)), None
            if len(vals) == 3:
                return ('pose', (vals[0], vals[1], vals[2])), None
            return None, u'坐标需要 2 或 3 个数（x,y[,yaw]），收到 %d 个' % len(vals)
        return ('name', text), None

    def _send_goal(self, x, y, yaw):
        """发送一个导航目标并等待结果。返回 (status, message, elapsed)。"""
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseGoal
        from actionlib_msgs.msg import GoalStatus

        if not self.server_ready:
            # 再给一次机会（可能刚连上）
            self.server_ready = self.client.wait_for_server(self.rospy.Duration(3.0))
        if not self.server_ready:
            return 'unavailable', u'move_base 不可用', 0.0

        # ★ 发目标前**先把目标坐标打进日志**。
        #
        # ⚠️ 为什么必须打（实测踩过）：
        #    原来不发这一句。任务失败时日志里只有
        #        [ERROR] Failed to get a plan.
        #    刷了几十万条，**完全不知道是哪个目标点失败的** ——
        #    查了几轮才定位到问题点位。
        #
        #    坐标是 ASCII，不会触发 Py2 日志编码问题。
        self.rospy.loginfo('[waypoint_nav] goal (%.2f, %.2f, %.2f) frame=%s',
                           float(x), float(y), float(yaw), self.frame_id)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = self.rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = 0.0
        # 只绕 Z 轴旋转
        goal.target_pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        t0 = time.time()
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(self.rospy.Duration(self.goal_timeout))
        elapsed = time.time() - t0

        if self.cancelled:
            self.client.cancel_goal()
            return 'cancelled', u'被取消', elapsed

        if not finished:
            self.client.cancel_goal()
            return 'timeout', u'导航超时（%.0f 秒）' % self.goal_timeout, elapsed

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return 'reached', u'已到达', elapsed
        if state == GoalStatus.REJECTED:
            return 'rejected', u'目标被拒绝（可能落在障碍物内或地图外）', elapsed
        if state == GoalStatus.ABORTED:
            return 'aborted', u'move_base 中止（路径被堵或无法规划）', elapsed
        if state == GoalStatus.PREEMPTED:
            return 'preempted', u'被抢占', elapsed
        return 'failed', u'未知状态码 %s' % state, elapsed

    def _goto_once(self, x, y, yaw, label):
        """带重试的导航。返回结果 dict。"""
        with self.lock:
            if self.busy:
                return {'ok': False, 'message': u'已有导航在进行中'}
            self.busy = True
            self.cancelled = False

        attempts = []
        try:
            for attempt in range(1, self.max_retries + 2):
                self._set_status(u'导航到 %s（第 %d/%d 次）目标 (%.2f, %.2f, %.2f)'
                                 % (label, attempt, self.max_retries + 1, x, y, yaw))
                status, message, elapsed = self._send_goal(x, y, yaw)
                attempts.append({'attempt': attempt, 'status': status,
                                 'message': _to_unicode(message),
                                 'elapsed': round(elapsed, 2)})

                if status == 'reached':
                    self.rospy.sleep(self.arrive_settle)
                    result = {
                        'ok': True, 'name': label, 'attempts': attempt,
                        'elapsed': round(elapsed, 2), 'status': status,
                        'message': _to_unicode(message),
                        'target': {'x': x, 'y': y, 'yaw': yaw},
                    }
                    self._publish_result(result)
                    self._set_status(u'已到达 %s（用时 %.1f 秒，尝试 %d 次）'
                                     % (label, elapsed, attempt))
                    return result

                if status in ('unavailable', 'cancelled'):
                    break
                # ★ 每次尝试失败都打一条日志（**这是本轮补的可观测性缺口**）。
                #
                # ⚠️ 原来失败时只把原因放进返回值 result，
                #    **一句日志都不打**。调用方（mission_manager）只记
                #       nav_failed_continue
                #    于是日志里完全看不出"是超时？被拒？还是路径堵了？"
                #    —— 调导航时等于瞎调。
                #
                #    这次就是卡在这：二维码区连试 3 次都失败，
                #    但 move_base 自己的日志里连一条 ERROR 都没有
                #    （配置压掉了），两边都看不到原因。
                #
                # 坐标和状态码都是 ASCII，不会触发 Py2 的日志编码问题。
                self.rospy.logwarn(
                    '[waypoint_nav] attempt %d/%d FAILED: status=%s '
                    'elapsed=%.1fs target=(%.2f, %.2f, %.2f) msg=%s',
                    attempt, self.max_retries + 1, status, elapsed,
                    x, y, yaw, message)
                if attempt <= self.max_retries:
                    self.rospy.sleep(self.retry_delay)

            total = sum(a['elapsed'] for a in attempts)
            result = {
                'ok': False, 'name': label, 'attempts': len(attempts),
                'elapsed': round(total, 2),
                'status': attempts[-1]['status'] if attempts else 'unknown',
                'message': attempts[-1]['message'] if attempts else u'未知错误',
                'attempts_detail': attempts,
                'target': {'x': x, 'y': y, 'yaw': yaw},
            }
            self._publish_result(result)
            self._set_status(u'导航到 %s 失败：%s' % (label, result['message']))
            return result
        finally:
            with self.lock:
                self.busy = False

    def _publish_result(self, result):
        from std_msgs.msg import String
        self.pub_result.publish(String(data=_u8(json.dumps(result, ensure_ascii=False))))

    # ------------------------------------------------------------------ 服务
    def on_goto(self, req):
        from std_srvs.srv import TriggerResponse

        kind, payload = self._parse_target(_to_unicode(getattr(req, 'message', '') or ''))
        if kind is None:
            return TriggerResponse(success=False, message=_u8(payload))
        if kind == 'name' and payload not in self.waypoints:
            return TriggerResponse(
                success=False,
                message=_u8(u'未知航点 "%s"。已配置：%s'
                            % (payload, u', '.join(self.waypoint_order) if self.waypoint_order
                               else u'（无，请检查 waypoints_file 参数）')))

        name = payload if kind == 'name' else u'(%.2f,%.2f,%.2f)' % payload
        if kind == 'name':
            x, y, yaw = self.waypoints[payload]
        else:
            x, y, yaw = payload

        if kind == 'name' and payload in self.waypoint_order:
            from std_msgs.msg import Int32
            self.pub_index.publish(Int32(data=self.waypoint_order.index(payload)))

        result = self._goto_once(x, y, yaw, name)
        return TriggerResponse(
            success=bool(result.get('ok')),
            message=_u8(json.dumps(result, ensure_ascii=False)))

    def on_capture(self, req):
        """把当前位姿记录为命名航点，写回航点文件。"""
        from std_srvs.srv import TriggerResponse

        name = _to_unicode(getattr(req, 'message', '') or '').strip()
        if not name:
            return TriggerResponse(success=False, message=_u8(u'请给出航点名称'))

        pose = self._current_pose()
        if pose is None:
            return TriggerResponse(success=False,
                                   message=_u8(u'无法读取当前位姿（TF 不通或定位未收敛）'))

        x, y, yaw = pose
        with self.lock:
            self.waypoints[name] = (x, y, yaw)
            if name not in self.waypoint_order:
                self.waypoint_order.append(name)

        saved = False
        if self.waypoints_file:
            try:
                import yaml
                data = {'frame_id': self.frame_id, 'waypoints': {}}
                for n in self.waypoint_order:
                    vx, vy, vyaw = self.waypoints[n]
                    data['waypoints'][n] = {'x': round(vx, 4),
                                            'y': round(vy, 4),
                                            'yaw': round(vyaw, 4)}
                with io.open(self.waypoints_file, 'w', encoding='utf-8') as fp:
                    fp.write(u'# 由 waypoint_nav.py 的 /nav/capture_pose 自动写入\n')
                    fp.write(u'# 格式：waypoints.<名称>.{x,y,yaw}\n')
                    fp.write(unicode(yaml.safe_dump(data, allow_unicode=True,
                                                    default_flow_style=False,
                                                    sort_keys=False)))
                saved = True
            except Exception as exc:
                self.rospy.logerr('[waypoint_nav] 写入航点文件失败：%s' % repr(exc))

        self._set_status(u'已记录航点 %s = (%.3f, %.3f, %.3f)%s'
                         % (name, x, y, yaw, u'，已写入文件' if saved else u'（仅内存）'))
        return TriggerResponse(
            success=True,
            message=_u8(json.dumps({'name': name, 'x': round(x, 4),
                                    'y': round(y, 4), 'yaw': round(yaw, 4),
                                    'saved_to_file': saved}, ensure_ascii=False)))

    def on_cancel(self, _req):
        from std_srvs.srv import TriggerResponse
        with self.lock:
            self.cancelled = True
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass
        self._set_status(u'已请求取消导航')
        return TriggerResponse(success=True, message=_u8(u'已取消'))

    def on_reload(self, _req):
        from std_srvs.srv import TriggerResponse
        self._load_waypoints(self.waypoints_file)
        return TriggerResponse(
            success=True,
            message=_u8(json.dumps({'count': len(self.waypoints),
                                    'waypoints': self.waypoint_order},
                                   ensure_ascii=False)))

    def on_status(self, _req):
        from std_srvs.srv import TriggerResponse
        with self.lock:
            busy = self.busy
        return TriggerResponse(
            success=True,
            message=_u8(json.dumps({
                'busy': busy,
                'move_base_ready': self.server_ready,
                'frame_id': self.frame_id,
                'goal_timeout': self.goal_timeout,
                'max_retries': self.max_retries,
                'waypoint_count': len(self.waypoints),
                'waypoints': self.waypoint_order,
            }, ensure_ascii=False)))


def main():
    setup_stdout()
    fix_ros_logging()
    import rospy
    rospy.init_node('waypoint_nav')
    try:
        WaypointNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
