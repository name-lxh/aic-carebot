#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
mission_manager.py — 智慧养老护理赛题：任务状态机（核心编排节点）

它做什么
--------
把赛题要求的一整条任务链串成**零人工干预**、**每步都有超时与降级**的状态机：

    自检 → 播报开始 → 导航到二维码区 → 识别二维码 → 解析巡视顺序
      → 客厅环境检测与播报 → 导航到 A 点 → 人脸核验门禁
      → 【按二维码顺序】逐房间执行任务（卧室/厨房/书房）
      → 返回随机起点 → 播报完成

为什么不用参考实现的 SMACH 示例
------------------------------
组委会的 `carebot_tasks` 包里的 SMACH 代码（`patrol_smach.py` 等）是 pi_robot 项目的
通用巡逻 demo，与赛题任务无关，且：
  * 目标点是写死的正方形巡逻点；
  * 没有超时、没有重试、没有降级；
  * 没有与二维码、物联网、人脸核验的任何耦合。

赛题的硬性约束是「**1 分钟内未对项目情景做出回应**即触发故障判定，再次出现按出局处理」，
所以本状态机的每个状态都强制带 timeout，超时后走「重试 → 降级 → 播报说明」路径，
**绝不无限等待**。

对外接口
--------
订阅：
  /eldercare/start        std_msgs/Bool      True 启动任务（裁判示意后触发）
  /eldercare/room_order   std_msgs/Int16MultiArray   手动注入房间顺序（调试/兜底）
  /eldercare/abort        std_msgs/Bool      True 中止任务

发布：
  /eldercare/mission_state  std_msgs/String  当前状态名（2 号机位展示）
  /eldercare/mission_event  std_msgs/String  结构化事件 JSON（供技术报告做数据分析）
  /eldercare/status         std_msgs/String  人类可读状态摘要

服务：
  /eldercare/run_once     std_srvs/Trigger   阻塞式跑完整轮任务（调试用）
  /eldercare/get_report   std_srvs/Trigger   返回本轮任务的结构化报告（JSON）

参数：
  ~waypoints_file     点位配置 yaml
  ~autostart          启动后是否立即开跑（默认 false，等 /eldercare/start）
  ~goal_timeout       单点导航超时（秒）
  ~max_retries        单步最大重试次数
  ~dry_run            true 时只跑逻辑不控实物（调试用）
"""

import sys
import json
import math
import os
import threading
import time

import actionlib
import rospy

# ★ 统一的日志修复（原来这里内联了一份 fix_ros_logging，
#   现在抽成 ros_logfix.py 共用，避免两份实现不同步）
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.join(
    _os.path.dirname(_os.path.abspath(__file__)), '..', 'scripts'))
try:
    from ros_logfix import fix_ros_logging, setup_stdout
except ImportError:
    def fix_ros_logging():
        pass
    def setup_stdout():
        pass
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse

try:
    unicode  # noqa: F821  (Python 2)
except NameError:
    unicode = str  # noqa: A001

try:
    import tf
    HAS_TF = True
except ImportError:
    HAS_TF = False


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


# ---------------------------------------------------------------------------
# 状态定义
# ---------------------------------------------------------------------------
IDLE = 'IDLE'
SELF_CHECK = 'SELF_CHECK'
ANNOUNCE_START = 'ANNOUNCE_START'
NAV_TO_QR_AREA = 'NAV_TO_QR_AREA'
QR_SCAN = 'QR_SCAN'
PARSE_ORDER = 'PARSE_ORDER'
ANNOUNCE_ORDER = 'ANNOUNCE_ORDER'
LIVING_ROOM_ENV = 'LIVING_ROOM_ENV'
NAV_TO_A_POINT = 'NAV_TO_A_POINT'
FACE_GATE = 'FACE_GATE'
ROOM_SEQUENCE = 'ROOM_SEQUENCE'
NAV_TO_START = 'NAV_TO_START'
ANNOUNCE_FINISH = 'ANNOUNCE_FINISH'
DONE = 'DONE'
ABORTED = 'ABORTED'

# 房间编号 -> (点位名, 中文名)
ROOM_INFO = {
    2: ('room_B', u'卧室'),
    3: ('room_C', u'厨房'),
    4: ('room_D', u'书房'),
}


class NavClient(object):
    """move_base 动作客户端封装：带超时、可取消、返回结构化结果。"""

    def __init__(self, goal_timeout):
        self.goal_timeout = goal_timeout
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.available = False

    def wait_for_server(self, timeout=30.0):
        rospy.loginfo('[mission] 等待 move_base 动作服务器...')
        self.available = self.client.wait_for_server(rospy.Duration(timeout))
        if self.available:
            rospy.loginfo('[mission] move_base 已连接')
        else:
            rospy.logerr('[mission] move_base 连接超时')
        return self.available

    def goto(self, x, y, yaw, frame='map', timeout=None):
        """导航到一个位姿。返回 (status, message)。

        status ∈ {'reached', 'timeout', 'rejected', 'aborted', 'unavailable'}
        """
        if not self.available:
            return 'unavailable', u'move_base 不可用'

        timeout = timeout or self.goal_timeout
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = 0.0
        # 仅绕 Z 轴旋转，用四元数表示
        goal.target_pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(timeout))

        if not finished:
            self.client.cancel_goal()
            return 'timeout', u'导航超时（%.0fs）' % timeout

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return 'reached', u'已到达'
        if state == GoalStatus.REJECTED:
            return 'rejected', u'目标被拒绝（可能位于障碍物内或代价地图外）'
        if state == GoalStatus.ABORTED:
            return 'aborted', u'导航失败（路径被阻塞或无法规划）'
        return 'aborted', u'导航结束，状态码 %d' % state

    def cancel(self):
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass


class EventLogger(object):
    """结构化事件日志：JSON Lines，供技术报告做量化分析。"""

    def __init__(self, path, enabled=True):
        self.path = path
        self.enabled = enabled and bool(path)
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.records = []
        if self.enabled:
            try:
                d = os.path.dirname(path)
                if d and not os.path.isdir(d):
                    os.makedirs(d)
                with open(self.path, 'wb') as fp:
                    fp.write(b'')
                rospy.loginfo('[mission] 事件日志：%s', self.path)
            except Exception as exc:
                rospy.logwarn('[mission] 无法创建事件日志 %s：%s', path, exc)
                self.enabled = False

    def log(self, state, event, detail=None, level='info'):
        rec = {
            't': round(time.time() - self.t0, 3),
            'wall': time.strftime('%H:%M:%S'),
            'state': state,
            'event': event,
            'level': level,
        }
        if detail:
            rec['detail'] = detail if isinstance(detail, (dict, list)) else _to_unicode(detail)
        with self.lock:
            self.records.append(rec)
            if self.enabled:
                try:
                    # ⚠️⚠️ 必须**以二进制追加 + 显式 UTF-8 编码**。
                    #
                    # 原来是：
                    #     with open(self.path, 'a') as fp:      # 文本模式
                    #         fp.write(json.dumps(rec, ensure_ascii=False) + '\n')
                    #
                    # Py2 下 `open()` 的文本模式默认 **ASCII** 编码。
                    # 而 `ensure_ascii=False` 会返回**含中文的 unicode** ——
                    # 写进去时抛
                    #     UnicodeEncodeError: 'ascii' codec can't encode ...
                    # 而外层是 `except Exception: pass`，**异常被静默吞掉**。
                    #
                    # 后果极其误导（我为此查了好几轮）：
                    #   事件日志**从第一条带中文的事件起就全部丢失**。
                    #   而第一条带中文的正是 `_nav_to` 里的
                    #       「导航到 二维码识别区域（第 1 次）目标 ...」
                    #   → 日志看起来"停在导航开始处"，
                    #     像是**导航卡死了**；实际任务在正常跑，
                    #     只是后面的记录写不进去。
                    #
                    # 同一个坑在 `telemetry`/`status` 上也踩过。
                    # **Py2 下凡是写文件，一律用二进制 + 显式编码。**
                    with open(self.path, 'ab') as fp:
                        fp.write((json.dumps(rec, ensure_ascii=False)
                                  + '\n').encode('utf-8'))
                except Exception as exc:
                    # 不再静默：写失败要说出来，否则又变成"看不到原因"
                    try:
                        rospy.logwarn_throttle(
                            5.0, '[mission] 事件日志写入失败: %s', exc)
                    except Exception:
                        pass

    def summary(self):
        ok = sum(1 for r in self.records if r['level'] == 'ok')
        warn = sum(1 for r in self.records if r['level'] == 'warn')
        err = sum(1 for r in self.records if r['level'] == 'error')
        return {
            'duration_sec': round(time.time() - self.t0, 2),
            'records': len(self.records),
            'ok': ok, 'warn': warn, 'error': err,
            'events': self.records,
        }


class MissionManager(object):

    def __init__(self):
        # --- 参数 -----------------------------------------------------------
        self.waypoints_file = rospy.get_param('~waypoints_file', '')
        self.autostart = bool(rospy.get_param('~autostart', False))
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 45.0))
        self.max_retries = int(rospy.get_param('~max_retries', 2))
        self.dry_run = bool(rospy.get_param('~dry_run', False))
        # ★ skip_services：真导航、不调服务（仿真用，见 _do_qr_scan 的说明）
        self.skip_services = bool(rospy.get_param('~skip_services', False))
        # 仿真里注入的二维码载荷（模拟"扫到了什么"）。
        # 空字符串 → _resolve_order 拿不到 → 走默认顺序 B,C,D
        self.qr_stub_payload = rospy.get_param('~qr_stub_payload', '')
        self.qr_timeout = float(rospy.get_param('~qr_timeout', 20.0))
        self.face_max_rounds = int(rospy.get_param('~face_max_rounds', 3))
        self.settle_time = float(rospy.get_param('~settle_time', 1.0))
        self.log_path = rospy.get_param('~event_log',
                                        '/tmp/carebot_eldercare/mission_events.jsonl')

        self.waypoints = self._load_waypoints(self.waypoints_file)

        # --- 运行时状态 -----------------------------------------------------
        self.state = IDLE
        self.room_order = []          # 例如 [2, 4, 3]
        self.current_room_idx = 0
        self.start_pose = None        # 任务开始时的位姿（用于返回随机起点）
        self.abort_requested = False
        self.running = False

        self.lock = threading.RLock()
        self.logger = EventLogger(self.log_path)

        # --- 组件 -----------------------------------------------------------
        self.nav = NavClient(self.goal_timeout)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.tf_listener = None
        if HAS_TF:
            try:
                self.tf_listener = tf.TransformListener()
            except Exception:
                pass

        # --- 发布 -----------------------------------------------------------
        self.pub_state = rospy.Publisher('/eldercare/mission_state', String,
                                         queue_size=10, latch=True)
        self.pub_event = rospy.Publisher('/eldercare/mission_event', String, queue_size=50)
        self.pub_status = rospy.Publisher('/eldercare/status', String, queue_size=20, latch=True)
        self.pub_room_order = rospy.Publisher('/eldercare/room_order', Int16MultiArray,
                                              queue_size=5, latch=True)

        # --- 订阅 -----------------------------------------------------------
        self.sub_start = rospy.Subscriber('/eldercare/start', Bool, self.on_start, queue_size=5)
        self.sub_order = rospy.Subscriber('/eldercare/room_order', Int16MultiArray,
                                          self.on_room_order, queue_size=5)
        self.sub_abort = rospy.Subscriber('/eldercare/abort', Bool, self.on_abort, queue_size=5)
        # 二维码节点的输出直接作为顺序来源
        self.sub_qr = rospy.Subscriber('/qr/room_order', Int16MultiArray,
                                       self.on_qr_order, queue_size=5)

        # --- 服务 -----------------------------------------------------------
        self.srv_run = rospy.Service('/eldercare/run_once', Trigger, self.on_run_once)
        self.srv_report = rospy.Service('/eldercare/get_report', Trigger, self.on_get_report)

        # --- 语音输出通道（无则静默）---------------------------------------
        self.voice_pub = rospy.Publisher('/voice/say', String, queue_size=20)

        self.logger.log(IDLE, 'manager_ready', {
            'waypoints_file': self.waypoints_file,
            'dry_run': self.dry_run,
        })
        self._set_state(IDLE, u'任务管理器就绪，等待启动信号')

        if self.autostart:
            rospy.Timer(rospy.Duration(3.0), self._autostart_cb, oneshot=True)

    # =================================================================== 工具
    @staticmethod
    def _load_waypoints(path):
        if not path or not os.path.isfile(path):
            rospy.logwarn('[mission] 点位文件不存在：%s，使用内置默认点位', path)
            return {
                'waypoints': {
                    'start_default': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                    'qr_area': {'x': 1.5, 'y': 0.0, 'yaw': 0.0},
                    'point_A': {'x': 1.0, 'y': -0.8, 'yaw': 1.57},
                    'room_B': {'x': 2.2, 'y': 1.2, 'yaw': 0.0},
                    'room_C': {'x': 2.2, 'y': -1.2, 'yaw': 0.0},
                    'room_D': {'x': 0.6, 'y': 1.4, 'yaw': 3.14},
                },
                'tolerances': {'goal_timeout_sec': 45, 'max_retries': 2},
            }
        try:
            import yaml
            with open(path, "rb") as fp:
                data = yaml.safe_load(fp.read()) or {}
            rospy.loginfo('[mission] 已加载点位：%s', path)
            return data
        except Exception as exc:
            rospy.logerr('[mission] 点位加载失败 %s：%s', path, exc)
            return {'waypoints': {}}

    def _wp(self, name):
        """取点位 (x, y, yaw)；不存在返回 None。"""
        wp = (self.waypoints.get('waypoints') or {}).get(name)
        if not wp:
            return None
        return (float(wp.get('x', 0.0)), float(wp.get('y', 0.0)), float(wp.get('yaw', 0.0)))

    def _wait_nav_ready(self, timeout=40.0):
        """等导航栈就绪再开跑。

        ⚠️ 为什么需要这一步（实测踩过）：

            `autostart` 是定时 3 秒后触发的。但那时导航栈**往往还没就绪**：

              · costmap 还没收到第一帧 /scan_filtered
                → 全局代价地图基本是空的 → 规划器报
                      Failed to get a plan
                  （一次能刷几十万条，把日志淹没）
              · amcl 粒子云还没收敛 → 起点估计飘

            结果"启动即失败"，而且失败信息看起来像"点位不可达"，
            很容易让人去查点位配置 —— 其实只是**没等够**。

        ⚠️ 实车上这个等待同样必要（甚至更重要）：
            amcl 需要几秒收敛、tf 树要建立、
            雷达驱动要出第一帧。赛题给 2 分钟准备时间，
            这里等几十秒完全划算。
        """
        self._set_status(u'等待导航栈就绪…')
        t0 = rospy.Time.now().to_sec()

        # 1) 等 /move_base action server
        #
        # ⚠️ 这里**不要**去读 `self.nav.server_ready` —— 那个标记是
        #    `waypoint_nav` **初始化时**设的：启动顺序上它往往早于
        #    move_base，于是永远停在 False，之后再没人更新。
        #    实测就是卡在这：日志里一句
        #        [mission] 等导航栈超时 40s
        #    然后照常开跑，接着 71 万条 "Failed to get a plan"。
        #
        #    正确做法：**直接问 actionlib 服务器在不在**。
        #    `wait_for_server` 会一直重试到超时，是权威判据。
        try:
            import actionlib
            from move_base_msgs.msg import MoveBaseAction
            client = actionlib.SimpleActionClient('move_base',
                                                  MoveBaseAction)
            if client.wait_for_server(rospy.Duration(timeout)):
                self._set_status(
                    u'move_base action server 就绪（%.1fs）'
                    % (rospy.Time.now().to_sec() - t0))
            else:
                self._set_status(u'move_base 未就绪，超时继续（可能失败）')
                rospy.logwarn('[mission] wait move_base timeout %.0fs',
                              timeout)
                return False
        except Exception as exc:
            # ⚠️ 不能写 u'...%r' % exc —— Py2 下 %r 产出 str（字节串），
            #    插进 unicode 格式串时按 **ASCII** 解码，
            #    异常消息含中文就抛 UnicodeDecodeError，
            #    而这行本身就在 except 里 —— 会把真异常也盖掉。
            #    （单元测试 test_no_bare_exception_in_unicode_format 抓到的）
            self._set_status(
                u'等 move_base 出错（继续）：%s' % _to_unicode(exc))

        # 2) 再等 costmap **稳定**（不是"有内容"就够了）
        #
        # ⚠️⚠️ 这里原来只检查"有没有占据格"，**不够** —— 这是
        #       "导航有时成功、有时连续失败"的根源。
        #
        # 原因：global_costmap 有 **static_layer**，
        #       它一开始就把静态地图里的墙填进去了。
        #       所以启动后不到 1 秒就能数到几百个障碍格 —— 检查通过。
        #       但**激光障碍层（obstacle_layer）还没扫完**，
        #       代价地图还在变。
        #
        #       实测：任务在启动 **3.1 秒**就判定"导航栈就绪"，
        #             然后在 5.5 秒发第一个目标 → 全局规划失败。
        #             而启动 **35 秒**后再去探测，
        #             洪水填充显示 **6 个点位全部可达**（连通性没问题）。
        #
        #       差别就是"代价地图有没有稳定下来"。
        #
        # 正确做法：**要求连续两次读到的障碍格数基本一致**，
        #           才算稳定。这样既不会早跑，也不会白等固定秒数。
        try:
            from nav_msgs.msg import OccupancyGrid
            STABLE_TOL = 5          # 两次之间障碍格允许的变化量
            STABLE_NEED = 2         # 需要连续几次一致
            MAX_READS = 20          # 最多读几次（每次间隔 1s）

            prev = None
            stable = 0
            n_occ = n_free = 0
            for _i in range(MAX_READS):
                if rospy.is_shutdown():
                    break
                try:
                    msg = rospy.wait_for_message(
                        '/move_base/global_costmap/costmap', OccupancyGrid,
                        timeout=8.0)
                except Exception:
                    break
                n_occ = sum(1 for v in msg.data if v >= 100)
                n_free = sum(1 for v in msg.data if v == 0)
                if prev is not None and abs(n_occ - prev) <= STABLE_TOL:
                    stable += 1
                    if stable >= STABLE_NEED:
                        break
                else:
                    stable = 0
                prev = n_occ
                rospy.sleep(1.0)

            self._set_status(
                u'代价地图稳定：%dx%d  障碍 %d 格  空闲 %d 格（读 %d 次）'
                % (msg.info.width, msg.info.height, n_occ, n_free, _i + 1))
            if n_occ == 0:
                rospy.logwarn('[mission] 代价地图里一个障碍都没有 —— '
                              '检查 /scan_filtered 是否在发布')
        except Exception as exc:
            # ⚠️ 同上：不能用 %r（见上面 wait move_base 那处的说明）
            self._set_status(
                u'读代价地图失败（继续）：%s' % _to_unicode(exc))

        # 3) 等 amcl 给出位姿
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped,
                                   timeout=15.0)
            self._set_status(u'amcl 已定位')
        except Exception:
            self._set_status(u'amcl 还没输出位姿（继续）')

        total = rospy.Time.now().to_sec() - t0
        self._set_status(u'导航栈就绪，共等待 %.1f 秒' % total)
        return True

    def _autostart_cb(self, _evt):
        rospy.loginfo('[mission] autostart 触发')
        # ★ 先等导航栈就绪，再开跑（见 _wait_nav_ready 的说明）
        threading.Thread(target=self._autostart_run).start()

    def _autostart_run(self):
        self._wait_nav_ready()
        self._start_mission()

    def _set_state(self, state, detail=None):
        with self.lock:
            self.state = state
        self.pub_state.publish(String(data=state))
        if detail:
            self._set_status(u'[%s] %s' % (state, detail))
        self.logger.log(state, 'state_enter', detail)

    def _set_status(self, text):
        self.pub_status.publish(String(data=_u8(text)))
        self.logger.log(self.state, 'status', text)
        rospy.loginfo('[mission] %s', _u8(text))

    def say(self, text, wait=False, timeout=20.0):
        """通过 voice_bridge 播报。"""
        self.voice_pub.publish(String(data=_u8(text)))
        self.logger.log(self.state, 'say', text)
        if wait:
            rospy.sleep(min(timeout, max(1.0, len(_to_unicode(text)) * 0.22)))

    def _check_abort(self):
        if self.abort_requested:
            raise MissionAborted()

    # ================================================================ 回调
    def on_start(self, msg):
        if msg.data and not self.running:
            threading.Thread(target=self._start_mission).start()

    def on_abort(self, msg):
        if msg.data:
            self.abort_requested = True
            self.nav.cancel()
            self._set_status(u'收到中止指令')

    def on_room_order(self, msg):
        order = [int(x) for x in msg.data if int(x) in ROOM_INFO]
        if order:
            with self.lock:
                self.room_order = order
            self._set_status(u'外部注入房间顺序：%s' % order)

    def on_qr_order(self, msg):
        order = [int(x) for x in msg.data if int(x) in ROOM_INFO]
        if order:
            with self.lock:
                self.room_order = order
            self._set_status(u'从二维码获得房间顺序：%s' % order)

    # ================================================================ 服务
    def on_run_once(self, _req):
        if self.running:
            return TriggerResponse(success=False, message=_u8(u'任务已在运行中'))
        ok = self._start_mission(blocking=True)
        summary = self.logger.summary()
        return TriggerResponse(
            success=ok,
            message=_u8(json.dumps({
                'result': 'success' if ok else 'failed',
                'duration_sec': summary['duration_sec'],
                'errors': summary['error'],
                'warnings': summary['warn'],
            }, ensure_ascii=False)))

    def on_get_report(self, _req):
        return TriggerResponse(success=True,
                               message=_u8(json.dumps(self.logger.summary(),
                                                      ensure_ascii=False)))

    # ================================================================ 主流程
    def _start_mission(self, blocking=False):
        with self.lock:
            if self.running:
                return False
            self.running = True
            self.abort_requested = False
        try:
            self._run_mission()
            return True
        except MissionAborted:
            self._set_state(ABORTED, u'任务被中止')
            return False
        except Exception as exc:
            self._set_state(ABORTED, u'任务异常终止：%s' % repr(exc))
            self.logger.log(self.state, 'exception', _to_unicode(exc), level='error')
            return False
        finally:
            with self.lock:
                self.running = False

    def _run_mission(self):
        self.logger.t0 = time.time()
        self.logger.log(IDLE, 'mission_begin', level='ok')

        # ---------------------------------------------------- 1. 自检
        self._set_state(SELF_CHECK)
        if not self._self_check():
            self.say(u'自检未通过，请检查设备后重试')
            self._set_state(ABORTED, u'自检失败')
            return

        # 记录起始位姿 —— 赛题起点位置随机，这是返回时的依据
        self.start_pose = self._current_pose()
        self.logger.log(SELF_CHECK, 'start_pose', self.start_pose, level='ok')

        # ---------------------------------------------------- 2. 播报开始
        self._set_state(ANNOUNCE_START)
        self.say(u'您好，我是养老护理机器人，现在开始执行居家巡视任务', wait=True)

        # ---------------------------------------------------- 3. 导航到二维码区
        self._set_state(NAV_TO_QR_AREA)
        if not self._nav_to('qr_area', u'二维码识别区域'):
            self.say(u'无法到达二维码区域，我将尝试就近识别')
            self.logger.log(NAV_TO_QR_AREA, 'nav_failed_continue', level='warn')

        # ---------------------------------------------------- 4. 识别二维码
        self._set_state(QR_SCAN)
        payload = self._do_qr_scan()

        # ---------------------------------------------------- 5. 解析巡视顺序
        self._set_state(PARSE_ORDER)
        order = self._resolve_order(payload)
        if not order:
            self.say(u'未能识别二维码中的巡视顺序，我将按卧室、厨房、书房的默认顺序巡视')
            order = [2, 3, 4]
            self.logger.log(PARSE_ORDER, 'fallback_default_order', order, level='warn')

        with self.lock:
            self.room_order = order
        msg = Int16MultiArray()
        msg.data = order
        self.pub_room_order.publish(msg)
        self.logger.log(PARSE_ORDER, 'room_order_final', order, level='ok')

        # ---------------------------------------------------- 6. 播报顺序
        self._set_state(ANNOUNCE_ORDER)
        names = u'、'.join(ROOM_INFO[x][1] for x in order)
        self.say(u'已读取巡视顺序：%s，我将依次巡视' % names, wait=True)

        # ---------------------------------------------------- 7. 客厅环境检测
        self._set_state(LIVING_ROOM_ENV)
        self._do_living_room_env()

        # ---------------------------------------------------- 8. 导航到 A 点
        self._set_state(NAV_TO_A_POINT)
        if not self._nav_to('point_A', u'A 点人脸核验区'):
            self.say(u'无法到达 A 点')
            self.logger.log(NAV_TO_A_POINT, 'nav_failed', level='error')
            # A 点是门槛条件，到不了就无法继续；但仍尝试一次门禁
        self._check_abort()

        # ---------------------------------------------------- 9. 人脸核验
        self._set_state(FACE_GATE)
        passed = self._do_face_gate()
        if not passed:
            self.say(u'未能确认主人身份，任务终止以保护居家安全', wait=True)
            self._set_state(ABORTED, u'人脸核验未通过')
            return

        # ---------------------------------------------------- 10. 按顺序巡视房间
        self._set_state(ROOM_SEQUENCE)
        for idx, room_id in enumerate(order):
            self._check_abort()
            self.current_room_idx = idx
            wp_name, cn_name = ROOM_INFO[room_id]
            self.say(u'现在前往第 %d 个房间：%s' % (idx + 1, cn_name), wait=True)

            if not self._nav_to(wp_name, cn_name):
                self.say(u'无法到达%s，跳过该房间' % cn_name)
                self.logger.log(ROOM_SEQUENCE, 'room_unreachable', cn_name, level='error')
                continue

            self._do_room_task(room_id)

        # ---------------------------------------------------- 11. 返回起点
        self._set_state(NAV_TO_START)
        self._do_return_to_start()

        # ---------------------------------------------------- 12. 播报完成
        self._set_state(ANNOUNCE_FINISH)
        summary = self.logger.summary()
        self.say(u'全部巡视任务已完成，总用时 %.0f 秒，谢谢' % summary['duration_sec'], wait=True)
        self.logger.log(ANNOUNCE_FINISH, 'mission_end', summary, level='ok')
        self._set_state(DONE, u'任务完成')

    # ---------------------------------------------------------------- 各步骤
    def _self_check(self):
        """检查关键子系统是否在线。返回 True 表示可以继续。"""
        checks = {}

        # move_base
        checks['move_base'] = self.nav.wait_for_server(timeout=20.0)

        # 关键话题
        try:
            topics = [t[0] for t in rospy.get_published_topics()]
        except Exception:
            topics = []
        for topic, label in (('/scan', u'激光雷达'),
                             ('/camera/rgb/image_raw', u'深度相机'),
                             ('/iot/status', u'物联网桥接')):
            checks[label] = topic in topics

        for k, v in checks.items():
            self.logger.log(SELF_CHECK, 'check_' + _to_unicode(k),
                            'ok' if v else 'missing',
                            level='ok' if v else 'warn')

        self._set_status(u'自检结果：%s' % json.dumps(
            dict((_to_unicode(k), bool(v)) for k, v in checks.items()), ensure_ascii=False))

        # move_base 是硬性依赖；其他缺失只告警不阻塞
        if not checks['move_base']:
            return False
        return True

    def _current_pose(self):
        """从 tf 读取当前位姿 (x, y, yaw)；失败返回 None。"""
        if self.tf_listener is None:
            return None
        try:
            self.tf_listener.waitForTransform('/map', '/base_footprint',
                                              rospy.Time(0), rospy.Duration(3.0))
            trans, rot = self.tf_listener.lookupTransform('/map', '/base_footprint',
                                                          rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            return {'x': round(trans[0], 3), 'y': round(trans[1], 3),
                    'yaw': round(euler[2], 3)}
        except Exception as exc:
            rospy.logwarn('[mission] 读取当前位姿失败：%s', exc)
            return None

    def _nav_to(self, wp_name, label, retries=None):
        """导航到已命名点位，带重试。返回是否到达。"""
        wp = self._wp(wp_name)
        if wp is None:
            self._set_status(u'点位 %s 未配置，跳过' % wp_name)
            self.logger.log(self.state, 'waypoint_missing', wp_name, level='warn')
            return False

        if self.dry_run:
            self._set_status(u'[dry_run] 跳过导航到 %s' % label)
            self.logger.log(self.state, 'dry_run_nav', wp_name)
            rospy.sleep(0.5)
            return True

        retries = self.max_retries if retries is None else retries
        x, y, yaw = wp

        for attempt in range(1, retries + 2):
            self._check_abort()
            self._set_status(u'导航到 %s（第 %d 次）目标 (%.2f, %.2f, %.2f)'
                             % (label, attempt, x, y, yaw))
            status, message = self.nav.goto(x, y, yaw)

            if status == 'reached':
                self.logger.log(self.state, 'nav_reached',
                                {'waypoint': wp_name, 'attempt': attempt}, level='ok')
                rospy.sleep(self.settle_time)
                return True

            self.logger.log(self.state, 'nav_failed',
                            {'waypoint': wp_name, 'attempt': attempt,
                             'status': status, 'message': message},
                            level='warn')
            if status == 'unavailable':
                return False
            rospy.sleep(0.5)

        return False

    def _do_qr_scan(self):
        """调用二维码识别服务，返回原始载荷（可能为 None）。

        ⚠️ 三种模式的区别（容易混，这里说清）：

            dry_run = true           完全干跑：**不导航**、不调服务
                                     → 只验证状态机流转

            skip_services = true     真导航、**不调服务**
                                     → 验证"导航 + 流程"这一层。
                                       仿真里没有摄像头/麦克风/HomeAssistant，
                                       但导航栈是完整的，所以这个模式最有用。

            两个都 false             完整链路（实车用）
        """
        if self.dry_run:
            self.logger.log(QR_SCAN, 'dry_run_qr', level='warn')
            return None
        if self.skip_services:
            # ★ 仿真模式：没有摄像头，用参数里指定的载荷（或走默认顺序）
            payload = _to_unicode(self.qr_stub_payload or '')
            self.logger.log(QR_SCAN, 'skip_services_qr',
                            payload or u'（空，将用默认顺序）', level='warn')
            self.say(u'（仿真）跳过二维码识别')
            return payload or None
        try:
            rospy.wait_for_service('/qr/detect_once', timeout=3.0)
            proxy = rospy.ServiceProxy('/qr/detect_once', Trigger)
            resp = proxy()
            if resp.success:
                # message 格式为 "2,3,4|原始载荷"
                parts = _to_unicode(resp.message).split('|', 1)
                payload = parts[1] if len(parts) > 1 else parts[0]
                self.logger.log(QR_SCAN, 'qr_detected', payload, level='ok')
                self.say(u'二维码读取成功')
                return payload
            self.logger.log(QR_SCAN, 'qr_timeout', _to_unicode(resp.message), level='warn')
            self.say(u'二维码识别超时，请将二维码正对摄像头')
            return None
        except Exception as exc:
            self.logger.log(QR_SCAN, 'qr_service_error', _to_unicode(exc), level='error')
            return None

    def _resolve_order(self, payload):
        """把二维码载荷解析成房间顺序；已在 qr_detector 中解析过一次，
        这里做二次兜底（也支持外部直接注入）。"""
        with self.lock:
            if self.room_order:
                return list(self.room_order)
        if not payload:
            return None
        try:
            from carebot_qr.nodes.qr_detector import parse_room_order
            order, _reason = parse_room_order(_to_unicode(payload))
            return order
        except Exception:
            pass
        # 内联兜底解析（避免包导入路径问题）
        import re
        text = _to_unicode(payload)
        digits = re.findall(r'\d', text)
        if len(digits) == 3:
            vals = [int(d) for d in digits]
            if set(vals) == {2, 3, 4}:
                return vals
        letters = re.findall(r'[BbCcDd]', text)
        if len(letters) == 3:
            vals = [{'B': 2, 'C': 3, 'D': 4}[c.upper()] for c in letters]
            if set(vals) == {2, 3, 4}:
                return vals
        return None

    def _do_living_room_env(self):
        """客厅环境检测：光照 + 温湿度，并语音播报。"""
        if self.dry_run:
            self.say(u'[调试] 模拟播报客厅环境数据')
            return

        data = None
        try:
            rospy.wait_for_service('/iot/refresh', timeout=5.0)
            proxy = rospy.ServiceProxy('/iot/refresh', Trigger)
            resp = proxy()
            self.logger.log(LIVING_ROOM_ENV, 'env_raw', _to_unicode(resp.message),
                            level='ok' if resp.success else 'warn')
            if not resp.success:
                self.say(u'环境传感器读取不完整，我将播报已获取的部分')
            data = json.loads(_to_unicode(resp.message))
        except Exception as exc:
            self.logger.log(LIVING_ROOM_ENV, 'iot_service_error', _to_unicode(exc), level='error')
            self.say(u'环境传感器暂时无法读取')
            return

        # 组织成自然语言播报
        env = data.get('env') or {}
        parts = []
        for key in ('illuminance', 'temperature', 'humidity'):
            item = env.get(key)
            if not isinstance(item, dict) or item.get('value') is None:
                continue
            desc = item.get('desc', key)
            unit = item.get('unit', '')
            parts.append(u'%s %.1f%s' % (desc, item['value'], unit))

        if parts:
            self.say(u'当前客厅环境：' + u'，'.join(parts))
            self.logger.log(LIVING_ROOM_ENV, 'env_announced', parts, level='ok')
        else:
            self.say(u'未能读取到有效的环境数据')
            self.logger.log(LIVING_ROOM_ENV, 'env_empty', level='warn')

    def _do_face_gate(self):
        """调用人脸核验服务，返回是否通过。"""
        if self.dry_run:
            self.logger.log(FACE_GATE, 'dry_run_face_gate', level='warn')
            self.say(u'[调试] 跳过人脸核验')
            return True
        if self.skip_services:
            # ★ 仿真模式：没有摄像头。
            #    ⚠️ 这里仍然**走"通过"的分支**而不是直接 return True ——
            #       因为要验证的是"核验通过之后流程能不能继续"，
            #       而不是"跳过核验"。日志里标明是跳过，不冒充真核验。
            self.logger.log(FACE_GATE, 'skip_services_face',
                            {'passed': True, 'note': u'仿真，无摄像头'},
                            level='warn')
            self.say(u'（仿真）跳过人脸核验')
            return True
        try:
            rospy.wait_for_service('/face/verify', timeout=3.0)
            proxy = rospy.ServiceProxy('/face/verify', Trigger)
            self.say(u'请在摄像头前停留，正在进行身份核验')
            resp = proxy()
            detail = _to_unicode(resp.message)
            self.logger.log(FACE_GATE, 'face_verify',
                            {'passed': bool(resp.success), 'detail': detail},
                            level='ok' if resp.success else 'warn')
            if resp.success:
                self.say(u'身份核验通过，欢迎回家')
                return True
            self.say(u'未识别到主人，无法进入房间')
            return False
        except Exception as exc:
            self.logger.log(FACE_GATE, 'face_service_error', _to_unicode(exc), level='error')
            # 服务不可用：赛题要求必须核验，这里保守判定为不通过
            self.say(u'人脸核验服务不可用')
            return False

    def _do_room_task(self, room_id):
        """执行单个房间的任务。"""
        wp_name, cn_name = ROOM_INFO[room_id]
        self.say(u'已到达%s' % cn_name)
        self.logger.log(ROOM_SEQUENCE, 'room_arrived', cn_name, level='ok')

        if room_id == 2:
            self._task_bedroom()
        elif room_id == 3:
            self._task_kitchen()
        elif room_id == 4:
            self._task_study()

    def _control(self, device, action, label, retries=None):
        """通过 /iot/control 服务控制设备，带重试。返回是否成功。"""
        if self.dry_run:
            self._set_status(u'[dry_run] %s' % label)
            self.logger.log(self.state, 'dry_run_control', label, level='warn')
            rospy.sleep(0.3)
            return True

        retries = self.max_retries if retries is None else retries
        for attempt in range(1, retries + 2):
            try:
                rospy.wait_for_service('/iot/control', timeout=3.0)
                # ★ 用自定义类型 Control，**不能用 std_srvs/Trigger**。
                #
                # ⚠️ 原来写的是：
                #       proxy = rospy.ServiceProxy('/iot/control', Trigger)
                #       payload = json.dumps({'device': ..., 'action': ...})
                #       resp = proxy(payload)
                #    而 **Trigger 的请求是空的**（TriggerRequest.__slots__ == []），
                #    `proxy(payload)` 的参数会被**静默丢弃**，
                #    服务端收到的是空请求 → 永远拿不到 device/action
                #    → 设备控制从未生效。
                #
                #    比赛里这是卧室电灯/窗帘(10分) + 厨房风扇(10分) = 20 分。
                #    症状隐蔽：服务能连、类型名对、编译不报错，
                #    只有真调一次并检查服务端拿到的内容才会暴露。
                from carebot_iot_msgs.srv import Control
                proxy = rospy.ServiceProxy('/iot/control', Control)
                resp = proxy(str(device), str(action), '')
                if resp.success:
                    self.logger.log(self.state, 'control_ok',
                                    {'device': device, 'action': action, 'attempt': attempt},
                                    level='ok')
                    return True
                self.logger.log(self.state, 'control_failed',
                                {'device': device, 'action': action,
                                 'message': _to_unicode(resp.message), 'attempt': attempt},
                                level='warn')
            except Exception as exc:
                self.logger.log(self.state, 'control_error',
                                {'device': device, 'action': action,
                                 'error': _to_unicode(exc), 'attempt': attempt},
                                level='error')
            rospy.sleep(0.5)
        return False

    def _task_bedroom(self):
        """卧室任务：打开电灯与窗帘（10 分）。"""
        ok_light = self._control('light_bedroom', 'on', u'打开卧室电灯')
        ok_curtain = self._control('curtain_bedroom', 'open', u'打开卧室窗帘')

        if ok_light and ok_curtain:
            self.say(u'已为您打开卧室的电灯和窗帘')
        elif ok_light:
            self.say(u'卧室电灯已打开，窗帘未能打开')
        elif ok_curtain:
            self.say(u'卧室窗帘已打开，电灯未能打开')
        else:
            self.say(u'卧室设备无响应，请检查智能家居网关')

    def _task_kitchen(self):
        """厨房任务：读取烟雾状态 → 控制传感器电源 → 打开风扇（5 + 10 分）。"""
        # 1) 读取烟雾浓度
        smoke_text = None
        if not self.dry_run:
            try:
                rospy.wait_for_service('/iot/refresh', timeout=3.0)
                proxy = rospy.ServiceProxy('/iot/refresh', Trigger)
                resp = proxy()
                data = json.loads(_to_unicode(resp.message))
                smoke = (data.get('env') or {}).get('smoke') or {}
                smoke_text = smoke.get('text')
                if smoke_text:
                    self.say(u'厨房烟雾检测：%s' % smoke_text)
                    self.logger.log(ROOM_SEQUENCE, 'smoke_read',
                                    {'text': smoke_text, 'value': smoke.get('value')},
                                    level='ok')
                else:
                    self.say(u'未能读取到厨房烟雾传感器数据')
                    self.logger.log(ROOM_SEQUENCE, 'smoke_read_failed', level='warn')
            except Exception as exc:
                self.logger.log(ROOM_SEQUENCE, 'smoke_error', _to_unicode(exc), level='error')
                self.say(u'烟雾传感器读取失败')

        # 2) 控制传感器电源（若为可断电型）
        self._control('sensor_power', 'on', u'打开厨房传感器电源')

        # 3) 打开风扇
        ok_fan = self._control('fan_kitchen', 'on', u'打开厨房风扇')
        if ok_fan:
            self.say(u'已打开厨房风扇，保持空气流通')
        else:
            self.say(u'厨房风扇未能打开，请检查继电器')

    def _task_study(self):
        """书房任务：漏水检测并播报（5 分）。"""
        if self.dry_run:
            self.say(u'[调试] 书房漏水检测正常')
            return
        try:
            rospy.wait_for_service('/iot/refresh', timeout=3.0)
            proxy = rospy.ServiceProxy('/iot/refresh', Trigger)
            resp = proxy()
            data = json.loads(_to_unicode(resp.message))
            leak = (data.get('env') or {}).get('water_leak') or {}

            if leak.get('configured') is False:
                self.say(u'书房漏水传感器尚未配置，请检查设备映射')
                self.logger.log(ROOM_SEQUENCE, 'leak_unconfigured', level='warn')
                return

            text = leak.get('text') or u'未知状态'
            abnormal = bool(leak.get('abnormal'))
            self.say(u'书房漏水检测：%s' % text)
            self.logger.log(ROOM_SEQUENCE, 'leak_read',
                            {'text': text, 'abnormal': abnormal, 'raw': leak.get('raw')},
                            level='warn' if abnormal else 'ok')

            # 创意增强：检测到漏水时联动关闭进水阀（若配置了该设备）
            if abnormal:
                self.say(u'检测到漏水，正在尝试关闭进水阀')
                self._control('water_valve', 'off', u'关闭进水阀')
        except Exception as exc:
            self.logger.log(ROOM_SEQUENCE, 'leak_error', _to_unicode(exc), level='error')
            self.say(u'漏水传感器读取失败')

    def _do_return_to_start(self):
        """返回起点。起点位置随机 ⇒ 用任务开始时记录的位姿。"""
        if self.start_pose:
            self._set_status(u'返回起点 (%.2f, %.2f, %.2f)'
                             % (self.start_pose['x'], self.start_pose['y'],
                                self.start_pose['yaw']))
            # 起点由 tf 得到，直接发送而不是靠命名点位
            if not self.dry_run:
                for attempt in range(1, self.max_retries + 2):
                    status, message = self.nav.goto(self.start_pose['x'],
                                                    self.start_pose['y'],
                                                    self.start_pose['yaw'])
                    if status == 'reached':
                        self.logger.log(NAV_TO_START, 'returned_to_start',
                                        {'attempt': attempt, 'pose': self.start_pose},
                                        level='ok')
                        self.say(u'已返回起点')
                        return True
                    self.logger.log(NAV_TO_START, 'return_failed',
                                    {'attempt': attempt, 'status': status,
                                     'message': message}, level='warn')
                self.say(u'未能精确返回起点，已尽力靠近')
            return True

        # 兜底：用命名点位
        self._set_status(u'未记录到起始位姿，使用默认起点')
        if self._nav_to('start_default', u'默认起点'):
            self.say(u'已返回起点')
            return True
        self.say(u'返回起点失败')
        return False


class MissionAborted(Exception):
    pass


if __name__ == '__main__':
    setup_stdout()
    fix_ros_logging()
    rospy.init_node('mission_manager')
    try:
        MissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
