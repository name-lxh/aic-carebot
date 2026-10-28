#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
fake_sensors.py — 假传感器仿真器（无硬件验证导航栈）

为什么需要这个
--------------
以前验证导航只能靠"编译通过 + launch 能解析"，但这**证明不了导航逻辑对不对**：
move_base 能不能规划出路径？costmap 有没有正确吃到激光？amcl 会不会收敛？
这些必须有**真实流动的传感器数据**才能测。

本节点模拟出导航栈需要的全部输入，让整条链路在没有机器人的情况下跑起来：
    · /odom           里程计（含噪声，可模拟漂移）
    · /imu/data       IMU
    · /scan           激光（从一张内存地图里做射线投射）
    · /tf             odom → base_footprint，以及各传感器的静态变换

这样就能：
    ✅ 验证 move_base 能否规划路径并产出 /cmd_vel
    ✅ 验证 costmap 是否正确读到障碍
    ✅ 验证 amcl 能否在给定地图上定位
    ✅ 验证任务状态机能完整走完状态流转
    ✅ 在改参数后立刻看效果，不用排队等实车

设计要点
--------
1. **运动模型可控**：订阅 /cmd_vel 并按它积分出位姿 —— 这样 move_base
   发出的速度指令会真的"驱动"机器人，形成闭环。
2. **激光来自地图**：用射线投射从一张 ASCII 地图算出扫描距离，
   障碍位置与地图一致，costmap 才能正确建立。
3. **噪声可关**：默认带一点噪声更接近真实；调试时可 --noise 0 关掉。
4. **不依赖 Gazebo**：虚拟机没有独显，Gazebo 跑不动。这个纯 Python 实现
   开销极小，2 核虚拟机也能流畅跑。

用法
----
    # 基本用法（起一个 8m×8m 的房间，中间有两个障碍）
    rosrun carebot_nav_kit fake_sensors.py

    # 指定起始位姿
    rosrun carebot_nav_kit fake_sensors.py _start_x:=1.0 _start_y:=1.0 _start_yaw:=0.0

    # 不开噪声（调试规划器时更干净）
    rosrun carebot_nav_kit fake_sensors.py _noise:=0.0

    # 用自定义地图文件（每行一串字符，# 是墙，. 是空地）
    rosrun carebot_nav_kit fake_sensors.py _map_file:=/path/to/room.txt
"""

import math
import os
import random
import sys
import threading
import time

import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String


try:
    unicode  # noqa: F821
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


# ---------------------------------------------------------------------------
# 默认房间地图
#   坐标原点在房间中心。'#' 是墙，'.' 是空地。
#   分辨率 0.2 m/格 → 8m × 8m
# ---------------------------------------------------------------------------
DEFAULT_MAP = [
    "##########################################",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#.........####...........................",
    "#.........####...........................",
    "#.........####...........................",
    "#.........####...........................",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#....................######..............",
    "#....................######..............",
    "#....................######..............",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "##########################################",
]


class FakeWorld(object):
    """一个二维栅格世界，支持射线投射算激光距离。"""

    def __init__(self, rows, res=0.2):
        self.rows = rows
        self.res = res
        self.h = len(rows)
        self.w = max(len(r) for r in rows)
        # 归一化行长度
        self.rows = [r.ljust(self.w, '#') for r in rows]
        # 世界坐标：原点在中心
        self.ox = -self.w * res / 2.0
        self.oy = -self.h * res / 2.0

    def is_occupied(self, wx, wy):
        """世界坐标是否撞墙。

        ⚠️⚠️ 这里必须**翻转 Y 轴**，否则机器人在仿真里会被"放进墙里"。

        踩坑记录（这个 bug 让导航完全跑不起来，而且症状全都指向别处）：

            `ARENA` 是**按画图顺序**写的 —— 第 0 行是**北边（y 最大）**，
            最后一行是南边。`build_grid()` 开的起点走廊在**最后两行**，
            `write_pgm()` 又按从上到下写出，所以 PGM 里走廊在**底部**，
            地图文件完全正确。

            但世界坐标系是"y 越大越靠北"。原来的映射是

                gy = int((wy - oy) / res)        # ← 没有翻转

            于是起点世界坐标 (0, -1.90) 被算成

                gy = int((-1.90 + 2.65) / 0.1) = 7      ← 北边，行 7

            而真正的起点走廊在 **gy = 46**（最后两行）。
            **机器人被判定站在第 7 行 —— 那是墙里。**

            后果：射线立刻打到"墙"，`/scan` 返回 0.2~0.5 m 的一圈假距离。
            这些点经 laser_filter（阈值 0.38）漏过去几个，
            被 costmap 标成障碍 → **机器人周围一圈假障碍** →
                DWA: Rotate recovery can't rotate in place. Cost: -1.00
                全局规划: aborted（"路径被阻塞或无法规划"）

            为什么极难查：
              · 日志只喊 aborted / Cost: -1.00，**完全不提坐标系**
              · 用洪水填充查可达性：**6 个点位全可达** ——
                因为代价地图是在**错误但自洽**的格子上算的
              · 激光自扫阈值从 0.22 提到 0.38 都没解决
                （因为那批 0.225 的点根本不是自扫，**是"墙"**）
              · 单看地图 PGM 完全正常，`sim_precheck` 也全过

            教训：**两套坐标系的 y 方向约定必须显式写明并互相校验。**
                  "地图文件是对的"不代表"读地图的代码是对的"。
        """
        gx = int((wx - self.ox) / self.res)
        # ★ Y 轴翻转：行 0 在**北**（y 最大），行 h-1 在南
        gy = self.h - 1 - int((wy - self.oy) / self.res)
        if gx < 0 or gy < 0 or gx >= self.w or gy >= self.h:
            return True
        return self.rows[gy][gx] == '#'

    def raycast(self, x, y, angle, max_range, step=0.02):
        """从 (x,y) 沿 angle 投射，返回撞墙距离或 max_range。

        步进式投射：简单但对本项目足够（地图结构简单、性能要求低）。
        虚拟机只有 2 核，用解析法反而复杂，没必要。
        """
        ca = math.cos(angle)
        sa = math.sin(angle)
        d = 0.0
        while d < max_range:
            d += step
            if self.is_occupied(x + ca * d, y + sa * d):
                return max(0.0, d - step)
        return max_range


class FakeSensors(object):

    def __init__(self):
        setup_stdout()

        # ---- 参数 ----
        self.rate = rospy.get_param('~rate', 20.0)              # 主循环 Hz
        self.scan_rate = rospy.get_param('~scan_rate', 10.0)    # 激光 Hz
        self.frame_odom = rospy.get_param('~odom_frame', 'odom')
        self.frame_base = rospy.get_param('~base_frame', 'base_footprint')
        self.frame_laser = rospy.get_param('~laser_frame', 'laser_link')
        self.frame_imu = rospy.get_param('~imu_frame', 'imu_link')

        self.start_x = rospy.get_param('~start_x', 0.0)
        self.start_y = rospy.get_param('~start_y', 0.0)
        self.start_yaw = rospy.get_param('~start_yaw', 0.0)

        self.noise = rospy.get_param('~noise', 0.01)
        # ---- 里程计误差模型（要贴近真机，见 _integrate 里的说明）----
        # ⚠️ 默认值按真机实测取：
        #    · 轮径/码盘标度误差 + 打滑 → 位移标度误差 ~2%
        #    · 旋转标度误差              → ~1.5%
        #    · 每帧零漂（20Hz）          → 很小，但不能为 0
        #      否则仿真里程计是"完美的"，AMCL 参数就测不准了
        self.odom_drift = rospy.get_param('~odom_drift', 0.02)
        self.odom_slip = rospy.get_param('~odom_slip', 0.015)
        self.odom_walk = rospy.get_param('~odom_walk', 0.0004)

        # ------------------------------------------------------------------
        # 激光参数 —— **必须与实车雷达一致**
        # ------------------------------------------------------------------
        #
        # 实车默认雷达是 **lslidar M10**（见 launch/include/laser.launch
        # 的 `driver` 参数默认值 `lslidar_m10`）。
        #
        # 参数来源：`lslidar_driver/src/lslidar_driver.cc` 里 M10 分支
        #
        #     points_size_ = 1008        ← 一圈 1008 个点
        #     msg.angle_min = 0
        #     msg.angle_max = 2π
        #     msg.angle_increment = 2π / 1008   = 0.006233 rad = 0.357°
        #     msg.range_min = 0.0        （launch 里的 min_range）
        #     msg.range_max = 100.0      （launch 里的 max_range）
        #
        # ⚠️ 第一版写的是 721 点 / 0.5°，注释还写"与 RPLIDAR A1M8
        #    实测一致"—— 那是**另一款雷达**的参数，跟实车对不上。
        #
        #    这个差异会真实影响结果：
        #      · 1008 点比 721 点密 40% → costmap 障碍轮廓更细
        #      · 0.357° 比 0.5° 细 → 窄通道里的可通行性判断不同
        #      · AMCL 的 likelihood_field 模型吃点数，密度不同
        #        收敛速度和定位精度都会变
        #
        #    仿真参数跟实车不一致，等于在测另一台机器。
        #
        # ⚠️ range_max 的取值要小心：
        #    驱动 launch 里写 100.0，但 M10 **物理量程是 12m**
        #    （100.0 只是个上限，实际不会有 >12m 的回波）。
        #    仿真按物理量程 12m 来，这样 laser_filter 的
        #    `upper_threshold: 8.0` 能真正起作用 ——
        #    跟实车一致。
        self.scan_min = rospy.get_param('~scan_min_range', 0.15)
        self.scan_max = rospy.get_param('~scan_max_range', 12.0)
        self.scan_points = rospy.get_param('~scan_points', 1008)
        self.scan_noise = rospy.get_param('~scan_noise', 0.01)
        # （scan_rate 在上面已经读过了，这里不重复）

        # M10 转速（rpm）—— 真机默认 600rpm = 10Hz，与 scan_rate 一致
        self.scan_rpm = rospy.get_param('~scan_rpm', 600.0)

        # ---- 地图 ----
        #
        # 两种世界可选：
        #   1. `~arena:=true`  → 比赛场地（4m×4m，含客厅/卧室/厨房/书房
        #                        和 A/B/C/D 点位）—— **仿真比赛流程用这个**
        #   2. 默认            → 通用测试场地（8m×8m 两个障碍块）
        #                        —— 验证导航栈本身能跑
        #
        # ⚠️ 为什么用"导入 sim_arena.py"而不是再写一份 ASCII 地图：
        #    两份地图一定会不同步 —— 改了一份忘了另一份，
        #    结果地图 yaml 和仿真世界对不上，机器人在仿真里撞墙。
        #    单一数据源（sim_arena.ARENA）从根上避免这个问题。
        #    （make_sim_map.py 和 DEFAULT_MAP 就是靠一个单元测试
        #      校验一致性的，属于补救措施。）
        res = 0.2
        map_file = rospy.get_param('~map_file', '')
        use_arena = bool(rospy.get_param('~arena', False))

        if use_arena:
            try:
                # ⚠️ launch 里没法给节点设 PYTHONPATH，
                #    所以这里自己把包的 scripts 目录找出来加进去。
                #    `rospack find` 在 rospy 初始化后一定可用。
                import rospkg
                pkg_dir = rospkg.RosPack().get_path('carebot_nav_kit')
                scr = os.path.join(pkg_dir, 'scripts')
                if scr not in sys.path:
                    sys.path.insert(0, scr)
                import sim_arena
                rows = [''.join(r) for r in sim_arena.build_grid()]
                res = sim_arena.RESOLUTION
                rospy.loginfo(
                    u'已加载**比赛场地**：%d 行，分辨率 %.2f m/格'
                    % (len(rows), res))
            except Exception as exc:
                # ⚠️ 不能用 %r：%r 产出**字节串**，插进 unicode
                #    格式串时 Py2 会按 ASCII 解码它 —— 异常消息含
                #    中文（比如 ImportError 里带路径）就崩。
                #    而这行**正在 except 里** —— 崩了就看不到
                #    "为什么加载场地失败"，只剩一句无信息量的报错。
                rospy.logerr(
                    u'加载比赛场地失败（%s）—— 退回通用测试场地。'
                    u'检查 carebot_nav_kit/scripts/sim_arena.py 在不在'
                    % _to_unicode(exc))
                rows = DEFAULT_MAP
        elif map_file and os.path.isfile(map_file):
            with open(map_file) as fp:
                rows = [l.rstrip('\n') for l in fp if l.strip()]
            res = float(rospy.get_param('~map_resolution', 0.2))
            rospy.loginfo(u'已加载地图文件：%s（%d 行）' % (map_file, len(rows)))
        else:
            rows = DEFAULT_MAP
        self.world = FakeWorld(rows, res=res)

        # ---- 状态 ----
        self.x = self.start_x
        self.y = self.start_y
        self.yaw = self.start_yaw
        # 里程计位姿（可带漂移，与真实位姿分离）
        self.ox = self.x
        self.oy = self.y
        self.oyaw = self.yaw

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_cmd_time = rospy.Time.now()

        self.lock = threading.Lock()

        # ---- 发布 ----
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_truth = rospy.Publisher('/sim/ground_truth', Odometry,
                                         queue_size=10)
        self.pub_info = rospy.Publisher('/sim/info', String, queue_size=1,
                                        latch=True)

        # ---- 订阅 ----
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=1)

        # ---- TF ----
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 静态变换：base → laser / imu
        #   实车上的安装位置（车载雷达在前方，IMU 在中心）
        self.laser_offset = (0.08, 0.0, 0.25)      # 前 8cm，高 25cm
        self.imu_offset = (0.0, 0.0, 0.10)

        rospy.loginfo(u'假传感器仿真器已启动')
        rospy.loginfo(u'  世界尺寸：%.1f × %.1f m（分辨率 %.2f m）'
                      % (self.world.w * self.world.res,
                         self.world.h * self.world.res,
                         self.world.res))
        rospy.loginfo(u'  起始位姿：x=%.2f y=%.2f yaw=%.2f'
                      % (self.start_x, self.start_y, self.start_yaw))
        rospy.loginfo(u'  激光：%d 点，量程 %.2f~%.2f m，%.0f Hz'
                      % (self.scan_points, self.scan_min, self.scan_max,
                         self.scan_rate))
        rospy.loginfo(u'  噪声：%.3f' % self.noise)
        rospy.loginfo(u'  里程计误差：位移标度 %.3f  旋转标度 %.3f  零漂 %.5f'
                      u'（按真机实测量级取的默认值）'
                      % (self.odom_drift, self.odom_slip, self.odom_walk))

    # ------------------------------------------------------------------
    def on_cmd(self, msg):
        with self.lock:
            self.vx = msg.linear.x
            self.vy = msg.linear.y
            self.vth = msg.angular.z
            self.last_cmd_time = rospy.Time.now()

    # ------------------------------------------------------------------
    def _step(self, dt):
        """按当前速度积分位姿（全向底盘）。"""
        with self.lock:
            vx, vy, vth = self.vx, self.vy, self.vth
            age = (rospy.Time.now() - self.last_cmd_time).to_sec()

        # 超过 1 秒没收到指令就停下 —— 与实车 STM32 的
        # "1000ms 无指令自动停止" 行为一致，避免仿真里机器人一直跑
        if age > 1.0:
            vx = vy = vth = 0.0

        # 全向底盘：机体速度直接变换到世界系
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        dx = (vx * c - vy * s) * dt
        dy = (vx * s + vy * c) * dt
        dyaw = vth * dt

        # 真值
        nx = self.x + dx
        ny = self.y + dy

        # 简易碰撞处理：撞墙就不动（真实底盘会被挡住）
        if not self.world.is_occupied(nx, ny):
            self.x = nx
            self.y = ny
        self.yaw = self._norm(self.yaw + dyaw)

        # 里程计：在真值基础上加噪声与漂移
        #
        # ⚠️⚠️ 这里原本只有**逐帧独立**的随机噪声（见 publish_odom 里的
        #     `self.ox + random.gauss(0, noise)`）—— 那模拟的是**抖动**，
        #     不是真实里程计的**漂移**。
        #
        #     区别很关键，而且踩过坑：
        #
        #       真实轮式里程计：每次都是"上一次的估计 + 本次位移×(1+误差)"
        #                       → 误差**累积**，跑得越远偏得越多
        #       原实现：        每帧都在**真值**上加独立噪声
        #                       → 没有累积，等于里程计"永远不偏，只是抖"
        #
        #     后果：AMCL 拿到近乎完美的里程计，而
        #     `amcl_params.yaml` 里 odom_alpha1..alpha4 = 0.2 是给
        #     **真有漂移**的里程计准备的（组委会原配置）。
        #     高 alpha + 干净里程计 → AMCL 过度不信任里程计 → 粒子乱飘：
        #
        #         定位误差 3.33 m / 51.8°，规划成功率 1/6
        #
        #     一度让我以为是 AMCL 参数配错了。把 alpha 降到 0.02 后
        #     成功率回到 6/6 —— 但那是**掩盖了仿真器的问题**：
        #     实车上里程计会漂，低 alpha 反而危险。
        #
        #     正确做法：**让仿真器的漂移贴近真实**，然后用实车的参数去测。
        #     所以这里加累积漂移项。
        #
        #     ~odom_drift    每米位移的里程计标度误差（无量纲）
        #                    真机实测约 1%~3%（轮径误差 + 打滑）
        #     ~odom_slip     每弧度旋转的标度误差
        #     ~odom_walk     每帧的随机游走（真机会有的零漂）
        self.ox += dx * (1.0 + self.odom_drift)
        self.oy += dy * (1.0 + self.odom_drift)
        self.oyaw = self._norm(self.oyaw + dyaw * (1.0 + self.odom_slip))

        # 累积随机游走 —— 真机里程计的零漂，随时间是 random walk
        self.ox += random.gauss(0, self.odom_walk)
        self.oy += random.gauss(0, self.odom_walk)
        self.oyaw = self._norm(self.oyaw + random.gauss(0, self.odom_walk))

    @staticmethod
    def _norm(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _quat(self, yaw):
        return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))

    # ------------------------------------------------------------------
    def publish_odom(self, now):
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = self.frame_odom
        o.child_frame_id = self.frame_base

        # 带噪声的里程计（给导航栈用）
        o.pose.pose.position.x = self.ox + random.gauss(0, self.noise)
        o.pose.pose.position.y = self.oy + random.gauss(0, self.noise)
        o.pose.pose.position.z = 0.0
        o.pose.pose.orientation = self._quat(
            self.oyaw + random.gauss(0, self.noise))

        o.twist.twist.linear.x = self.vx
        o.twist.twist.linear.y = self.vy
        o.twist.twist.angular.z = self.vth

        # 协方差：给个合理值，否则 EKF 会不信任里程计
        for i in (0, 7, 14, 21, 28, 35):
            o.pose.covariance[i] = 0.01
            o.twist.covariance[i] = 0.01

        self.pub_odom.publish(o)

        # 真值（给测试脚本对照用）
        t = Odometry()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_truth'
        t.pose.pose.position.x = self.x
        t.pose.pose.position.y = self.y
        t.pose.pose.orientation = self._quat(self.yaw)
        self.pub_truth.publish(t)

        # TF: odom → base
        self.tf_broadcaster.sendTransform(
            (self.ox, self.oy, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, self.oyaw),
            now, self.frame_base, self.frame_odom)

    def publish_tf_static(self, now):
        # base → laser
        self.tf_broadcaster.sendTransform(
            self.laser_offset,
            tf.transformations.quaternion_from_euler(0, 0, 0),
            now, self.frame_laser, self.frame_base)
        # base → imu
        self.tf_broadcaster.sendTransform(
            self.imu_offset,
            tf.transformations.quaternion_from_euler(0, 0, 0),
            now, self.frame_imu, self.frame_base)

    def publish_scan(self, now):
        s = LaserScan()
        s.header.stamp = now
        s.header.frame_id = self.frame_laser

        n = self.scan_points
        # ⚠️ M10 是**整圈 360°**：angle_min=0, angle_max=2π，
        #    angle_increment = 2π/1008（见 lslidar_driver.cc）。
        #
        #    第一版写的是 angle_min=-π / angle_max=+π，
        #    虽然覆盖范围一样，但**零点位置不同** ——
        #    0 号点在正后方 vs 正前方，会让"激光与车头朝向"的关系反 180°。
        #    对 costmap 影响不大（整圈都用），但**对 amcl 有影响**：
        #    它按角度索引做似然场匹配，零点错位会让匹配整体偏。
        #    跟真机对齐：从正前方 0° 开始，逆时针到 2π。
        s.angle_min = 0.0
        s.angle_max = 2.0 * math.pi
        s.angle_increment = 2.0 * math.pi / float(n)
        # 每个点之间的时间增量（雷达旋转扫描，不是瞬时全采）
        s.scan_time = 1.0 / self.scan_rate
        s.time_increment = s.scan_time / float(n)
        s.range_min = self.scan_min
        s.range_max = self.scan_max

        # 雷达装在车体前方，扫描原点要算上偏移
        c = math.cos(self.yaw)
        sn = math.sin(self.yaw)
        lx = self.x + c * self.laser_offset[0]
        ly = self.y + sn * self.laser_offset[0]

        ranges = []
        for i in range(n):
            ang = s.angle_min + i * s.angle_increment
            d = self.world.raycast(lx, ly, self.yaw + ang, self.scan_max)
            if self.scan_noise > 0:
                d += random.gauss(0, self.scan_noise)
            # ★ 无回波必须发 inf，**不能发 0.0**。
            #
            # ⚠️ 这里踩了一个大坑，代价是"move_base 完全规划不出路径"：
            #
            #    原代码是 `d = 0.0`，注释写"ROS 约定 inf/0 都表示无回波"——
            #    **这个假设是错的**。真实雷达（RPLIDAR/lslidar）无回波时
            #    发的是 `inf`；发 `0.0` 会被下游当成"距离 0 米处有障碍"。
            #
            #    后果链条：
            #      1. 室内场景大量射线打不到墙 → 一大批 0.0
            #      2. laser_filter 的 lower_threshold=0.22
            #         → 0.0 被替换成……**0.22**（夹到阈值下限）
            #         —— 正好是"车体半径 0.18 + 4cm 余量"的位置
            #      3. costmap 的 obstacle_range=3.0 → 0.22 < 3.0
            #         → **机器人周围一圈全被标成障碍**
            #      4. 全局规划器认为起点就在障碍里 → 报
            #             NO PATH!
            #             Failed to get a plan from potential
            #             when a legal potential was found.
            #         而且**刷了 12 万条**，因为每次重规划都失败。
            #
            #    症状很容易查错方向：日志只喊 "NO PATH"，
            #    完全不提激光；而 /scan 的 0.0 看着又"像正常值"。
            #    真正的判据是 **/scan_filtered 里出现 0.22** ——
            #    那说明有一堆 0.0 被夹上来了。
            #
            # 修法：无回波发 inf。laser_filters 的 RangeFilter 对 inf
            #      的处理是"保持 inf"（不在 [lower, upper] 区间内且
            #      不是有限值），costmap 的 inf_is_valid=false 会忽略它。
            if d < self.scan_min or d >= self.scan_max:
                d = float('inf')
            ranges.append(d)

        s.ranges = ranges
        self.pub_scan.publish(s)

    def publish_imu(self, now):
        m = Imu()
        m.header.stamp = now
        m.header.frame_id = self.frame_imu
        m.orientation = self._quat(self.yaw + random.gauss(0, self.noise * 2))
        m.angular_velocity.z = self.vth + random.gauss(0, self.noise)
        for i in (0, 4, 8):
            m.orientation_covariance[i] = 0.001
            m.angular_velocity_covariance[i] = 0.001
            m.linear_acceleration_covariance[i] = 0.001
        self.pub_imu.publish(m)

    # ------------------------------------------------------------------
    def run(self):
        r = rospy.Rate(self.rate)
        dt = 1.0 / self.rate
        scan_every = max(1, int(round(self.rate / self.scan_rate)))
        i = 0

        # 发布一条启动信息，测试脚本可以等它
        info = String()
        info.data = (u'fake_sensors ready world=%.1fx%.1f start=(%.2f,%.2f,%.2f)'
                     % (self.world.w * self.world.res,
                        self.world.h * self.world.res,
                        self.start_x, self.start_y, self.start_yaw))
        self.pub_info.publish(info)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._step(dt)
            self.publish_odom(now)
            self.publish_tf_static(now)
            self.publish_imu(now)
            if i % scan_every == 0:
                self.publish_scan(now)
            i += 1
            r.sleep()


def main():
    setup_stdout()
    fix_ros_logging()
    rospy.init_node('fake_sensors', anonymous=False)
    try:
        node = FakeSensors()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        # ⚠️ Python 2 陷阱：不要写 `rospy.logerr(u'异常：%s' % repr(exc))`。
        #    u'' % exc 会对异常对象做 unicode() 转换，而异常消息是**字节串**，
        #    Python 2 会用 ASCII 解码它 —— 于是为了报告一个异常，
        #    反而抛出 UnicodeEncodeError，把真正的异常彻底掩盖。
        #    实测踩过：只看到 "File ..., line 557, in main" 后面一片空白。
        #
        #    正确做法：用 repr() 拿到 ASCII 安全的字节串再拼。
        _safe = repr(exc)
        try:
            rospy.logerr('fake_sensors failed: %s' % _safe)
        except Exception:
            pass
        # traceback 也要走 ASCII 安全的流
        import traceback
        traceback.print_exc()
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
