#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
nav_check.py — 导航栈自检工具

用途
----
在机器人排队时间宝贵的前提下，把"上车后才发现问题"变成"上车前就查出来"。
本工具检查导航栈能否正常工作的**全部前提条件**，并给出针对性的修复建议。

检查项
------
  1. 核心话题      /scan /odom /tf /cmd_vel 是否存在且在发布
  2. 数据频率      雷达、里程计、IMU 的发布频率是否合理
  3. TF 链路       map→odom→base_footprint→laser_link 是否连通
  4. 激光健康度    最小距离是否 > 车体半径（防"把自己当障碍"）
  5. 传感器时效    数据是否是"活的"（不是卡住的旧消息）
  6. 配置一致性    三处 robot_radius 是否一致
  7. 导航栈         amcl / move_base 是否就绪
  8. 动态测试       可选：实际发一个速度指令，验证底盘会动

用法
----
    # 完整检查（机器人已 bringup）
    rosrun carebot_nav_kit nav_check.py

    # 只做静态检查，等 30 秒收集数据（默认 10 秒）
    rosrun carebot_nav_kit nav_check.py --wait 30

    # 简要模式（只打结论，不出详细表）—— bringup.launch 里用的就是这个
    rosrun carebot_nav_kit nav_check.py --brief

    # 包含运动测试（会真的让机器人动一下！确认周围安全再用）
    rosrun carebot_nav_kit nav_check.py --motion-test

    # 输出 JSON 报告
    rosrun carebot_nav_kit nav_check.py --json /tmp/navcheck.json

退出码
------
    0 = 全部通过（可能有警告）
    1 = 存在阻塞性问题
"""

import argparse
import io
import json
import math
import os
import sys
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


OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'
COLORS = {OK: '\033[32;1m', WARN: '\033[33;1m', FAIL: '\033[31;1m'}
RESET = u'\033[0m'


class Report(object):
    def __init__(self, brief=False):
        self.items = []
        self.brief = brief

    def add(self, level, cat, name, detail='', fix=''):
        self.items.append({'level': level, 'category': cat, 'name': name,
                           'detail': detail, 'fix': fix})
        if self.brief and level != FAIL:
            return
        color = COLORS.get(level, '')
        line = u'  %s[%s]%s %-34s %s' % (color, level, RESET, name, detail)
        print(line)
        if fix and level != OK:
            print(u'         \033[35m→ 修复：%s\033[0m' % fix)

    def ok(self, cat, name, detail=''):
        self.add(OK, cat, name, detail)

    def warn(self, cat, name, detail='', fix=''):
        self.add(WARN, cat, name, detail, fix)

    def fail(self, cat, name, detail='', fix=''):
        self.add(FAIL, cat, name, detail, fix)

    def count(self, level):
        return sum(1 for i in self.items if i['level'] == level)

    def summary(self):
        return {'ok': self.count(OK), 'warn': self.count(WARN), 'fail': self.count(FAIL)}

    def category(self, cat):
        if not self.brief:
            print(u'\n\033[36;1m── %s %s' % (cat, RESET))


# ---------------------------------------------------------------------------
# 参数读取
# ---------------------------------------------------------------------------
def get_param(name, default=None):
    """读 ROS 参数，不存在时返回默认值。"""
    try:
        import rospy
        if rospy.has_param(name):
            return rospy.get_param(name)
    except Exception:
        pass
    return default


# ---------------------------------------------------------------------------
# 各项检查
# ---------------------------------------------------------------------------
def check_topics(rep, timeout=10.0):
    """检查核心话题是否存在且活跃。"""
    import rospy

    rep.category(u'1. 核心话题')

    # (话题, 说明, 是否必需, 缺失时的修复建议)
    topics = [
        ('/scan',        u'激光雷达（原始）',   False,
         u'雷达驱动未启动。检查：roslaunch carebot_nav_kit bringup.launch laser_driver:=lslidar_m10'),
        ('/scan_filtered', u'激光雷达（滤波后）', True,
         u'laser_filter 未启动。检查 include/laser_filter.launch 是否被包含'),
        ('/odom',        u'里程计',            True,
         u'底盘驱动未启动，或 EKF relay 未生效。检查 base_driver 参数与 bringup 输出'),
        ('/odometry/filtered', u'EKF 融合里程计', False,
         u'EKF 未启动。可先用 use_ekf:=false 跳过，或检查 ekf_params.yaml'),
        ('/imu',         u'IMU',               False,
         u'IMU 驱动未启动。EKF 融合会退化，但导航仍可工作'),
        ('/tf',          u'TF 变换',           True,
         u'没有任何节点发布 TF。静态 TF 是否启动？start_static_tf:=true'),
        ('/cmd_vel',     u'速度指令',          False,
         u'正常现象：没有节点在发布速度指令。move_base 启动后会发布'),
    ]

    published = {}
    try:
        published = dict((t[0], t[1]) for t in rospy.get_published_topics())
    except Exception as exc:
        rep.fail(u'核心话题', u'话题枚举', u'失败：%s' % repr(exc),
                 u'roscore 是否在跑？rosnode list 确认')
        return published

    for topic, label, required, fix in topics:
        if topic in published:
            rep.ok(u'核心话题', u'%s (%s)' % (label, topic),
                   u'类型 %s' % (published[topic][0] if published[topic] else u'?'))
        elif required:
            rep.fail(u'核心话题', u'%s (%s)' % (label, topic), u'不存在', fix)
        else:
            rep.warn(u'核心话题', u'%s (%s)' % (label, topic), u'不存在', fix)

    return published


def check_rates(rep, wait=10.0):
    """采样话题频率，判断数据是否是"活的"。"""
    import rospy

    rep.category(u'2. 数据频率与时效')

    targets = [
        ('/scan',              10.0, 3.0,   u'雷达'),
        ('/scan_filtered',     10.0, 3.0,   u'滤波后雷达'),
        ('/odom',               5.0, 1.0,   u'里程计'),
        ('/odometry/filtered',  5.0, 1.0,   u'融合里程计'),
        ('/imu',               20.0, 5.0,   u'IMU'),
    ]

    counts = {}
    subscribers = []

    def make_cb(key):
        def _cb(_msg):
            counts[key] = counts.get(key, 0) + 1
        return _cb

    try:
        from sensor_msgs.msg import LaserScan, Imu
        from nav_msgs.msg import Odometry
        msg_types = {
            '/scan': LaserScan, '/scan_filtered': LaserScan,
            '/odom': Odometry, '/odometry/filtered': Odometry,
            '/imu': Imu,
        }
    except ImportError as exc:
        rep.warn(u'数据频率', u'消息类型导入', u'失败：%s' % repr(exc))
        return

    published = [t[0] for t in rospy.get_published_topics()]
    for topic, _exp, _min, _label in targets:
        if topic not in published or topic not in msg_types:
            continue
        try:
            subscribers.append(
                rospy.Subscriber(topic, msg_types[topic], make_cb(topic), queue_size=20))
        except Exception:
            pass

    rospy.sleep(wait)

    for sub in subscribers:
        try:
            sub.unregister()
        except Exception:
            pass

    for topic, exp, min_hz, label in targets:
        if topic not in published:
            continue
        n = counts.get(topic, 0)
        hz = n / float(wait)
        if n == 0:
            rep.fail(u'数据频率', u'%s (%s)' % (label, topic), u'%.1f 秒内收不到任何消息' % wait,
                     u'话题存在但没有数据：驱动崩溃或串口打不开。查看该节点终端输出')
        elif hz < min_hz:
            rep.warn(u'数据频率', u'%s (%s)' % (label, topic),
                     u'%.1f Hz（预期约 %.0f Hz）' % (hz, exp),
                     u'频率偏低通常是 CPU 负载高或串口波特率配置错误')
        else:
            rep.ok(u'数据频率', u'%s (%s)' % (label, topic), u'%.1f Hz' % hz)


def check_scan_health(rep):
    """检查激光数据本身是否健康 —— 尤其是"会不会把自己当障碍"。"""
    import rospy
    from sensor_msgs.msg import LaserScan

    rep.category(u'3. 激光数据健康度')

    try:
        msg = rospy.wait_for_message('/scan_filtered', LaserScan, timeout=8.0)
    except Exception:
        rep.warn(u'激光健康度', u'读取 /scan_filtered', u'超时',
                 u'laser_filter 未启动？先确认 /scan_filtered 存在')
        return None

    # --- 有效点统计 ---
    valid = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0]
    total = len(msg.ranges)
    if total == 0:
        rep.fail(u'激光健康度', u'扫描点数', u'扫描数组为空',
                 u'雷达驱动异常，检查串口与波特率')
        return msg

    valid_ratio = len(valid) / float(total)
    rep.ok(u'激光健康度', u'有效点数',
           u'%d / %d（%.0f%%）' % (len(valid), total, valid_ratio * 100))
    if valid_ratio < 0.2:
        rep.warn(u'激光健康度', u'有效点比例偏低', u'仅 %.0f%%' % (valid_ratio * 100),
                 u'雷达被遮挡、镜面反射，或最小距离过滤过激')

    if valid:
        min_r = min(valid)
        max_r = max(valid)
        rep.ok(u'激光健康度', u'测距范围',
               u'%.2f m ~ %.2f m' % (min_r, max_r))

        # --- 关键检查：会不会扫到自己 ---
        robot_radius = 0.18
        try:
            r = rospy.get_param('/move_base/global_costmap/robot_radius', None)
            if r is None:
                r = rospy.get_param('/move_base/local_costmap/robot_radius', None)
            if r is not None:
                robot_radius = float(r)
        except Exception:
            pass

        if min_r < robot_radius:
            rep.fail(
                u'激光健康度', u'最小测距 < 车体半径',
                u'最小 %.3f m < 车体半径 %.3f m —— 雷达扫到了自己！' % (min_r, robot_radius),
                u'把 laser_filter_chain.yaml 的 lower_threshold 提到 %.2f 以上，'
                u'然后重启 laser_filter' % (robot_radius + 0.04))
        elif min_r < robot_radius + 0.03:
            rep.warn(u'激光健康度', u'最小测距接近车体半径',
                     u'%.3f m vs 半径 %.3f m，余量不足 3cm' % (min_r, robot_radius),
                     u'建议 lower_threshold 至少设到 %.2f' % (robot_radius + 0.04))
        else:
            rep.ok(u'激光健康度', u'无自遮挡',
                   u'最小 %.3f m > 半径 %.3f m（余量 %.3f m）'
                   % (min_r, robot_radius, min_r - robot_radius))

    # --- 角度覆盖 ---
    if msg.angle_increment > 0:
        coverage = math.degrees(msg.angle_increment * total)
        rep.ok(u'激光健康度', u'角度覆盖', u'%.0f°（%d 点，增量 %.3f°）'
               % (coverage, total, math.degrees(msg.angle_increment)))
        if coverage < 300:
            rep.warn(u'激光健康度', u'角度覆盖不足', u'仅 %.0f°' % coverage,
                     u'雷达可能只扫了一部分，检查驱动配置')

    # --- 坐标系 ---
    rep.ok(u'激光健康度', u'frame_id', msg.frame_id)
    return msg


def check_tf_tree(rep, timeout=8.0):
    """检查 TF 链路是否连通 —— 这是导航能工作的根本前提。"""
    import rospy
    import tf

    rep.category(u'4. TF 链路')

    listener = tf.TransformListener()
    rospy.sleep(2.0)   # 等 TF 缓冲

    # (父, 子, 说明, 是否必需)
    chains = [
        ('/odom', '/base_footprint', u'里程计 → 车体', True),
        ('/base_footprint', '/base_link', u'车体投影 → 车体中心', True),
        ('/base_link', '/laser_link', u'车体 → 雷达', True),
        ('/map', '/odom', u'地图 → 里程计（定位）', False),
        ('/base_link', '/IMU_link', u'车体 → IMU', False),
    ]

    results = {}
    for parent, child, label, required in chains:
        try:
            listener.waitForTransform(parent, child, rospy.Time(0),
                                      rospy.Duration(timeout))
            listener.lookupTransform(parent, child, rospy.Time(0))
            rep.ok(u'TF 链路', u'%s → %s（%s）' % (parent, child, label))
            results[child] = True
        except Exception:
            if required:
                fix = u''
                if child == '/base_footprint' and parent == '/odom':
                    fix = (u'底盘未发布 odom→base_footprint，且 EKF 也没发。'
                           u'检查底盘驱动的 is_pub_odom_tf 参数，或启动 EKF')
                elif child == '/base_link':
                    fix = u'静态 TF 未启动。检查 start_static_tf 参数'
                elif child == '/laser_link':
                    fix = u'静态 TF 未启动，或激光 frame_id 不叫 laser_link'
                rep.fail(u'TF 链路', u'%s → %s（%s）' % (parent, child, label),
                         u'变换不可用', fix)
            else:
                fix = u''
                if parent == '/map':
                    fix = (u'amcl 未收敛或未启动。导航时需要它；'
                           u'仅建图时不需要（gmapping 会发 map→odom）')
                elif child == '/IMU_link':
                    fix = u'IMU 未启动。若不用 IMU 可忽略'
                rep.warn(u'TF 链路', u'%s → %s（%s）' % (parent, child, label),
                         u'变换不可用', fix)
            results[child] = False

    # --- 检查是否有多个发布者抢同一个 TF（会导致抖动）---
    try:
        import rosgraph
        master = rosgraph.Master('/nav_check')
        pubs, _, _ = master.getSystemState()
        tf_publishers = []
        for topic, nodes in pubs:
            if topic in ('/tf', '/tf_static'):
                tf_publishers.extend(nodes)
        if tf_publishers:
            rep.ok(u'TF 链路', u'TF 发布者', u'%d 个节点：%s'
                   % (len(set(tf_publishers)), ', '.join(sorted(set(tf_publishers)))))
            static_pubs = [n for n in tf_publishers if 'static' in n.lower()]
            if len(static_pubs) > 4:
                rep.warn(u'TF 链路', u'静态 TF 发布者偏多', u'%d 个' % len(static_pubs),
                         u'确认没有重复发布同一对 frame（比如同时用了组委会 '
                         u'car_nav.launch 和本包 static_tf.launch）')
    except Exception:
        pass

    return results


def check_config_consistency(rep):
    """检查三处 robot_radius 是否一致 —— 不一致会导致规划与实际不符。"""
    import rospy

    rep.category(u'5. 配置一致性')

    # robot_radius 出现在三处：
    #   · costmap_common_params.yaml（全局+局部共用）
    #   · base.yaml（文档/参考）
    radii = {}
    for ns in ('/move_base/global_costmap/robot_radius',
               '/move_base/local_costmap/robot_radius'):
        try:
            if rospy.has_param(ns):
                radii[ns] = float(rospy.get_param(ns))
        except Exception:
            pass

    try:
        if rospy.has_param('/robot_profile/robot/geometry/radius'):
            radii['base.yaml'] = float(rospy.get_param('/robot_profile/robot/geometry/radius'))
    except Exception:
        pass

    if not radii:
        rep.warn(u'配置一致性', u'robot_radius', u'读不到任何 robot_radius 参数',
                 u'move_base 未启动属正常；导航时再看')
        return

    values = set(round(v, 4) for v in radii.values())
    detail = u', '.join(u'%s=%.3f' % (k.split('/')[-2] if '/' in k else k, v)
                        for k, v in radii.items())
    if len(values) == 1:
        rep.ok(u'配置一致性', u'robot_radius 一致', detail)
    else:
        rep.fail(u'配置一致性', u'robot_radius 不一致', detail,
                 u'三处必须相同，否则代价地图膨胀与实际车体不符，'
                 u'表现为"规划的路走不了"或"能过的地方不敢过"')

    # 检查 base_frame 命名是否统一（base_link vs base_footprint 混用是常见坑）
    frames = {}
    for ns in ('/move_base/global_costmap/robot_base_frame',
               '/move_base/local_costmap/robot_base_frame',
               '/amcl/base_frame_id'):
        try:
            if rospy.has_param(ns):
                frames[ns] = rospy.get_param(ns)
        except Exception:
            pass
    if frames:
        vals = set(frames.values())
        detail = u', '.join(u'%s=%s' % (k.split('/')[-1], v) for k, v in frames.items())
        if len(vals) == 1:
            rep.ok(u'配置一致性', u'base_frame 统一', detail)
        else:
            rep.fail(u'配置一致性', u'base_frame 不统一', detail,
                     u'costmap / amcl 必须用同一个基准坐标系，否则定位与规划错位')


def check_nav_stack(rep):
    """检查 amcl / move_base 是否就绪。"""
    import rospy

    rep.category(u'6. 导航栈')

    # --- move_base 动作服务 ---
    try:
        from move_base_msgs.msg import MoveBaseAction
        import actionlib
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if client.wait_for_server(rospy.Duration(5.0)):
            rep.ok(u'导航栈', u'move_base 动作服务', u'就绪')
        else:
            rep.warn(u'导航栈', u'move_base 动作服务', u'5 秒内未就绪',
                     u'导航 launch 是否启动？建图时不需要 move_base')
    except Exception as exc:
        rep.warn(u'导航栈', u'move_base 动作服务', u'检查失败：%s' % repr(exc))

    # --- amcl ---
    try:
        nodes = rospy.get_master().getSystemState()[0]
        node_names = [n for group in nodes for n in group]
        has_amcl = any('amcl' in str(n) for n in node_names)
        if has_amcl:
            rep.ok(u'导航栈', u'amcl 定位节点', u'已运行')
            rep.ok(u'导航栈', u'amcl 初始位姿',
                   u'提示：需在 rviz 用 2D Pose Estimate 给出初始位姿')
        else:
            rep.warn(u'导航栈', u'amcl 定位节点', u'未运行',
                     u'导航需要 amcl；仅建图不需要')
    except Exception:
        pass

    # --- 地图 ---
    try:
        from nav_msgs.msg import OccupancyGrid
        msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5.0)
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        rep.ok(u'导航栈', u'地图 /map',
               u'%d×%d 像素，分辨率 %.3f m，实际 %.1f×%.1f m'
               % (w, h, res, w * res, h * res))
        # 赛题场地 3m×3m，地图不该小得离谱
        if w * res < 1.0 or h * res < 1.0:
            rep.warn(u'导航栈', u'地图尺寸可疑', u'小于 1m',
                     u'地图可能建坏了，或 map_server 加载了错误文件')
    except Exception:
        rep.warn(u'导航栈', u'地图 /map', u'5 秒内未收到',
                 u'map_server 未启动（建图时正常；导航时必须启动）')


def check_motion(rep, cmd_pub, duration=1.5, speed=0.15):
    """动态测试：实际发一个速度指令，看里程计有没有变化。"""
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    rep.category(u'7. 运动测试（实测底盘响应）')

    print(u'  \033[33;1m⚠ 即将让机器人前进 %.1f 秒（速度 %.2f m/s），请确认周围安全！%s'
          % (duration, speed, RESET))

    try:
        start = rospy.wait_for_message('/odom', Odometry, timeout=5.0)
    except Exception:
        rep.fail(u'运动测试', u'读取初始里程计', u'超时', u'/odom 没有数据')
        return

    p0 = (start.pose.pose.position.x, start.pose.pose.position.y)

    twist = Twist()
    twist.linear.x = speed
    rate = rospy.Rate(20)
    t_end = time.time() + duration
    while time.time() < t_end and not rospy.is_shutdown():
        cmd_pub.publish(twist)
        rate.sleep()

    # 停车
    cmd_pub.publish(Twist())
    rospy.sleep(1.0)

    try:
        end = rospy.wait_for_message('/odom', Odometry, timeout=5.0)
    except Exception:
        rep.warn(u'运动测试', u'读取结束里程计', u'超时')
        return

    p1 = (end.pose.pose.position.x, end.pose.pose.position.y)
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    expected = speed * duration

    rep.ok(u'运动测试', u'里程计位移', u'%.3f m（发出指令约 %.3f m）' % (moved, expected))

    if moved < 0.01:
        rep.fail(u'运动测试', u'机器人没动', u'位移仅 %.4f m' % moved,
                 u'排查顺序：'
                 u'1) rostopic echo /cmd_vel 确认指令发出；'
                 u'2) 底盘串口是否打开（ls -l /dev/stm32）；'
                 u'3) 底盘电源是否打开；'
                 u'4) 底盘是否因超时保护自动停车（1000ms 无指令会停）')
    elif moved < expected * 0.5:
        rep.warn(u'运动测试', u'位移明显偏小', u'%.3f m vs 预期 %.3f m' % (moved, expected),
                 u'可能原因：地面打滑、速度标定系数不准、电池电量低。'
                 u'用 scripts/calibrate_linear.py 标定')
    elif moved > expected * 2.0:
        rep.warn(u'运动测试', u'位移明显偏大', u'%.3f m vs 预期 %.3f m' % (moved, expected),
                 u'速度标定系数需要修正。用 scripts/calibrate_linear.py 标定')
    else:
        rep.ok(u'运动测试', u'位移在合理范围',
               u'偏差 %.0f%%' % (abs(moved - expected) / expected * 100))


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    setup_stdout()

    ap = argparse.ArgumentParser(description=u'导航栈自检工具')
    ap.add_argument('--wait', type=float, default=10.0,
                    help=u'数据采样时长（秒），默认 10')
    ap.add_argument('--brief', action='store_true', help=u'简要模式（只打结论与问题）')
    ap.add_argument('--motion-test', action='store_true',
                    help=u'执行运动测试（会真的让机器人动！）')
    ap.add_argument('--json', default='', help=u'输出 JSON 报告路径')
    args = ap.parse_args(rospy.myargv()[1:])

    import rospy

    try:
        rospy.init_node('nav_check', anonymous=True, disable_signals=True)
    except Exception as exc:
        print(u'\033[31;1m无法连接 ROS Master：%s\033[0m' % repr(exc))
        print(u'请先确认 roscore 在跑，并且已 source 工作空间：')
        print(u'  source /opt/ros/kinetic/setup.bash')
        print(u'  source ~/omniWheelCareRobot/rosCode/devel/setup.bash')
        return 1

    rep = Report(brief=args.brief)

    print(u'\033[36;1m')
    print(u'=' * 70)
    print(u'  导航栈自检 (carebot_nav_kit)')
    print(u'  时间：%s' % time.strftime('%Y-%m-%d %H:%M:%S'))
    print(u'=' * 70)
    print(RESET)

    published = check_topics(rep)
    check_rates(rep, wait=args.wait)
    check_scan_health(rep)
    check_tf_tree(rep)
    check_config_consistency(rep)
    check_nav_stack(rep)

    if args.motion_test:
        try:
            from geometry_msgs.msg import Twist
            cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
            rospy.sleep(1.0)
            check_motion(rep, cmd_pub)
        except Exception as exc:
            rep.warn(u'运动测试', u'执行失败', u'%s' % repr(exc))
    else:
        rep.category(u'7. 运动测试')
        print(u'  \033[33m[跳过] 如需实测底盘响应，加 --motion-test（注意安全）%s' % RESET)

    # --- 汇总 ---
    s = rep.summary()
    print(u'\n' + u'=' * 70)
    color = COLORS[FAIL] if s['fail'] else (COLORS[WARN] if s['warn'] else COLORS[OK])
    print(u'%s  结果：通过 %d 项，警告 %d 项，失败 %d 项  %s'
          % (color, s['ok'], s['warn'], s['fail'], RESET))
    print(u'=' * 70)

    if s['fail']:
        print(u'\n\033[31;1m存在阻塞性问题，导航很可能无法正常工作。请先按上面的「→ 修复」处理。\033[0m')
    elif s['warn']:
        print(u'\n\033[33;1m没有阻塞性问题，但有警告项，建议逐条确认。\033[0m')
    else:
        print(u'\n\033[32;1m全部检查通过，导航栈可以正常工作。\033[0m')

    if args.json:
        try:
            with io.open(args.json, 'w', encoding='utf-8') as fp:
                fp.write(unicode(json.dumps({'summary': s, 'items': rep.items},
                                            ensure_ascii=False, indent=2)))
            print(u'\nJSON 报告已写入：%s' % args.json)
        except Exception as exc:
            print(u'\n写入 JSON 失败：%s' % repr(exc))

    return 1 if s['fail'] else 0


if __name__ == '__main__':
    sys.exit(main())
