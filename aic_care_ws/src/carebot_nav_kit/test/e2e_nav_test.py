#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
e2e_nav_test.py — 端到端导航测试（无硬件，用假传感器）

验证什么
--------
把整条导航链路**真的跑起来**：

    假传感器(/odom /scan /imu /tf)
        → laser_filter(/scan → /scan_filtered)
        → costmap
        → amcl（在地图上定位）
        → move_base（全局规划 + 局部规划）
        → /cmd_vel
        → 假传感器按指令运动
        → 到达目标点 ✔

只有走通这条闭环，才能说"导航配置是对的"。仅验证"编译通过 + launch
能解析"是不够的 —— 那证明不了 DWA 能算出路、costmap 能吃到障碍。

关键设计：用真实的 navigation.launch
------------------------------------
本测试**不自己拼节点**，而是启动 `navigation.launch`，原因：
  · navigation.launch 里有完整的 rosparam 加载（costmap / planner / amcl）
  · 自己起节点的话很容易漏参数，测的就不是真实配置了
    （踩过：手动起 move_base 时 base_frame_id 用了默认的 base_link，
      而项目统一用 base_footprint，结果 TF 找不到、costmap 起不来）

假传感器与真实驱动的区别只是数据来源，下游完全一致。

用法
----
    python2 e2e_nav_test.py                 # 完整测试
    python2 e2e_nav_test.py --keep          # 测完保留进程（人工观察）
    python2 e2e_nav_test.py --timeout 90    # 改单点超时

⚠️ ROS Melodic 的核心库只有 python2.7，必须用 python2 运行。
"""

from __future__ import division, print_function

import argparse
import io
import math
import os
import subprocess

# 字节串插进 unicode 格式串在 Py2 下会按 ASCII 解码而崩
def to_text(v):
    if v is None:
        return u''
    if isinstance(v, bytes):
        try:
            return v.decode('utf-8')
        except Exception:
            return v.decode('utf-8', 'replace')
    return u'%s' % (v,)

import sys
import time


# ---------------------------------------------------------------------------
def setup_stdout():
    """让 stdout/stderr 能输出中文（含字节串容忍）—— 详见各节点的同名实现。

    ⚠️ 要点：Python 2 的 traceback / logging 会往流里写**字节串**，
       标准 codecs.StreamWriter 收到字节串会先 decode('ascii') 而崩，
       所以必须自定义 writer 同时接受 bytes 与 unicode。
    """
    import codecs as _codecs

    class _Utf8Writer(_codecs.StreamWriter):
        def write(self, s):
            if s is None:
                return
            try:
                if isinstance(s, bytes):
                    self.stream.write(s)
                    return
                self.stream.write(s.encode('utf-8', 'backslashreplace'))
            except Exception:
                try:
                    self.stream.write(repr(s))
                except Exception:
                    pass

    def _wrap(stream):
        if stream is None:
            return stream
        buf = getattr(stream, 'buffer', None)
        if buf is None:
            buf = stream
        try:
            return _Utf8Writer(buf, errors='backslashreplace')
        except Exception:
            return stream

    sys.stdout = _wrap(sys.stdout)
    sys.stderr = _wrap(sys.stderr)


setup_stdout()

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry

try:
    unicode  # noqa: F821
except NameError:
    unicode = str  # noqa: A001


GREEN = u'\033[32;1m'
RED = u'\033[31;1m'
YELLOW = u'\033[33;1m'
CYAN = '\033[36;1m'
R = u'\033[0m'

PROCS = []


def say(msg):
    print(msg)
    sys.stdout.flush()


def ok(msg):
    say(u'  %s✔%s %s' % (GREEN, R, msg))


def err(msg):
    say(u'  %s✘%s %s' % (RED, R, msg))


def warn(msg):
    say(u'  %s⚠%s %s' % (YELLOW, R, msg))


def step(msg):
    say(u'\n%s▶ %s%s' % (CYAN, msg, R))


def spawn(cmd, log):
    f = open(log, 'w')
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT)
    PROCS.append((p, f, log))
    return p


def cleanup():
    for p, _f, _log in PROCS:
        try:
            if p.poll() is None:
                p.terminate()
        except Exception:
            pass
    time.sleep(2.0)
    for p, f, _log in PROCS:
        try:
            if p.poll() is None:
                p.kill()
        except Exception:
            pass
        try:
            f.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
class NavTester(object):

    def __init__(self, pkg, map_yaml, goal_timeout=90.0):
        self.pkg = pkg
        self.map_yaml = map_yaml
        self.goal_timeout = goal_timeout
        self.truth = None
        self.costmap = None
        self.odom = None

    # ------------------------------------------------------------ 工具
    def wait_for(self, cond, timeout, interval=0.5):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if rospy.is_shutdown():
                return False
            try:
                if cond():
                    return True
            except Exception:
                pass
            time.sleep(interval)
        return False

    def on_truth(self, msg):
        self.truth = msg

    def on_costmap(self, msg):
        self.costmap = msg

    def on_odom(self, msg):
        self.odom = msg

    def count_obstacles(self):
        if self.costmap is None:
            return -1
        return sum(1 for v in self.costmap.data if v >= 90)

    def topics(self):
        return set(t[0] for t in rospy.get_published_topics())

    # ------------------------------------------------------------ 启动
    def launch_stack(self):
        step(u'0. 启动导航栈（真实 navigation.launch + 假传感器）')

        spawn(['roscore'], '/tmp/e2e_roscore.log')
        t0 = time.time()
        while time.time() - t0 < 30:
            try:
                import rosgraph
                if rosgraph.is_master_online():
                    break
            except Exception:
                pass
            time.sleep(0.5)
        else:
            err(u'roscore 起不来')
            return False
        ok(u'roscore 就绪')

        # 假传感器：必须最先起，否则 EKF/costmap 初始化时会因缺 TF 报警
        spawn(['rosrun', 'carebot_nav_kit', 'fake_sensors.py',
               '_rate:=20.0', '_scan_rate:=10.0', '_noise:=0.005'],
              '/tmp/e2e_sensors.log')
        ok(u'假传感器（/odom /scan /imu /tf）')

        # 真实导航栈：rviz 关掉（虚拟机里没必要），waypoint_nav 关掉
        # （本测试直接给 move_base 发目标，不走航点服务）
        spawn(['roslaunch', 'carebot_nav_kit', 'navigation.launch',
               'base_driver:=none',
               'laser_driver:=none',
               'imu_driver:=none',
               'use_ekf:=false',
               'rviz:=false',
               'waypoint_nav:=false',
               'initial_x:=0.0',
               'initial_y:=0.0',
               'initial_yaw:=0.0',
               'map_file:=' + self.map_yaml],
              '/tmp/e2e_navstack.log')
        ok(u'navigation.launch（map_server / amcl / move_base / laser_filter）')
        return True

    # ------------------------------------------------------------ 导航
    def goto(self, x, y, yaw=0.0):
        cli = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not cli.wait_for_server(rospy.Duration(20.0)):
            err(u'move_base action server 没起来')
            return False, 0.0, None

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        say(u'    目标 (%.2f, %.2f, yaw=%.2f)，超时 %.0fs'
            % (x, y, yaw, self.goal_timeout))
        t0 = time.time()
        cli.send_goal(goal)

        last = 0.0
        while time.time() - t0 < self.goal_timeout:
            st = cli.get_state()
            if st in (3, 4, 5, 9):
                break
            el = time.time() - t0
            if el - last > 5.0 and self.truth is not None:
                p = self.truth.pose.pose.position
                say(u'      [%.1fs] 真值位置 (%.2f, %.2f)  距目标 %.2f m'
                    % (el, p.x, p.y, math.hypot(p.x - x, p.y - y)))
                last = el
            time.sleep(0.2)

        dur = time.time() - t0
        st = cli.get_state()
        final = None
        if self.truth is not None:
            p = self.truth.pose.pose.position
            final = (p.x, p.y)
        return (st == 3), dur, final

    # ------------------------------------------------------------ 主流程
    def run(self):
        say(u'\n' + u'=' * 70)
        say(u'  端到端导航测试（假传感器 + 真实 navigation.launch）')
        say(u'=' * 70)

        if not self.launch_stack():
            return 2

        rospy.init_node('e2e_nav_test', anonymous=True, disable_signals=True)
        rospy.Subscriber('/sim/ground_truth', Odometry, self.on_truth)
        rospy.Subscriber('/move_base/global_costmap/costmap',
                         OccupancyGrid, self.on_costmap)
        rospy.Subscriber('/odom', Odometry, self.on_odom)

        # ---- 等系统就绪 ----
        step(u'1. 等待系统稳定')
        if self.wait_for(lambda: self.truth is not None, 25):
            ok(u'假传感器数据在流')
        else:
            err(u'收不到 /sim/ground_truth')
            self.dump_log('/tmp/e2e_sensors.log')
            return 2

        if self.wait_for(lambda: self.costmap is not None, 60):
            ok(u'global costmap 已发布（%dx%d, %.3f m/格）'
               % (self.costmap.info.width, self.costmap.info.height,
                  self.costmap.info.resolution))
        else:
            err(u'global costmap 没出现 —— move_base 可能没起来')
            self.dump_log('/tmp/e2e_navstack.log')
            return 2

        # ---- 话题完整性 ----
        step(u'2. 检查话题链路')
        need = ['/odom', '/scan', '/scan_filtered', '/cmd_vel', '/map',
                '/amcl_pose', '/move_base/global_costmap/costmap']
        missing = []
        for t in need:
            if self.wait_for(lambda t=t: t in self.topics(), 8):
                ok(t)
            else:
                err(u'%s 缺失' % t)
                missing.append(t)
        if missing:
            warn(u'缺 %d 个话题，后续可能失败' % len(missing))

        # ---- 障碍 ----
        step(u'3. 检查 costmap 是否读到障碍')
        self.wait_for(lambda: self.count_obstacles() > 0, 20)
        n = self.count_obstacles()
        if n > 0:
            ok(u'global costmap 有 %d 个障碍格（激光→costmap 链路通）' % n)
        else:
            err(u'costmap 无任何障碍 —— 激光或地图有问题')
            return 2

        # ---- amcl 定位 ----
        step(u'4. 检查 amcl 是否定位成功')
        def _amcl_ready():
            try:
                rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped,
                                       timeout=3.0)
                return True
            except Exception:
                return False

        got_pose = self.wait_for(_amcl_ready, 40)
        if got_pose:
            ok(u'amcl 在发布 /amcl_pose（定位收敛）')
        else:
            warn(u'/amcl_pose 没收到（amcl 可能还在收敛）')

        # ---- 目标点 ----
        goals = [(1.5, -1.5, 0.0), (-2.0, 1.0, 0.0), (0.0, 0.0, 0.0)]
        results = []
        for i, (gx, gy, gyaw) in enumerate(goals, 1):
            step(u'%d. 目标点 %d：往 (%.1f, %.1f)' % (4 + i, i, gx, gy))
            reached, dur, final = self.goto(gx, gy, gyaw)
            if reached:
                d = math.hypot(final[0] - gx, final[1] - gy) if final else -1
                ok(u'到达，耗时 %.1f s，真值误差 %.3f m' % (dur, d))
                results.append((True, dur, d))
            else:
                err(u'未到达（耗时 %.1f s）' % dur)
                if final:
                    say(u'      最终真值位置 (%.2f, %.2f)' % final)
                results.append((False, dur, -1))

        # ---- 汇总 ----
        n_ok = sum(1 for r in results if r[0])
        say(u'')
        say(u'=' * 70)
        if n_ok == len(results):
            say(u'  %s结论：导航闭环完全正常（%d/%d 个目标点全部到达）%s'
                % (GREEN, n_ok, len(results), R))
            say(u'    假传感器 → laser_filter → costmap → amcl → move_base')
            say(u'    → /cmd_vel → 假传感器运动 → 到达目标')
            rc = 0
        elif n_ok > 0:
            say(u'  %s结论：部分成功（%d/%d）%s'
                % (YELLOW, n_ok, len(results), R))
            rc = 1
        else:
            say(u'  %s结论：导航闭环没跑通%s' % (RED, R))
            self.dump_log('/tmp/e2e_navstack.log')
            rc = 2
        say(u'=' * 70)
        return rc

    def dump_log(self, path, n=25):
        say(u'      --- %s（尾部 %d 行）---' % (os.path.basename(path), n))
        try:
            with io.open(path, encoding='utf-8', errors='replace') as fp:
                for line in fp.readlines()[-n:]:
                    say(u'      ' + line.rstrip())
        except Exception as exc:
            say(u'      （读不到：%s）' % repr(exc))


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkg-dir', default=None)
    ap.add_argument('--map', default=None)
    ap.add_argument('--timeout', type=float, default=90.0,
                    help=u'单个目标点的超时秒数')
    ap.add_argument('--keep', action='store_true')
    args = ap.parse_args()

    pkg = args.pkg_dir
    if not pkg:
        try:
            out = subprocess.Popen(['rospack', 'find', 'carebot_nav_kit'],
                                   stdout=subprocess.PIPE).communicate()[0]
            pkg = out.decode('utf-8').strip()
        except Exception:
            pkg = ''
    if not pkg or not os.path.isdir(pkg):
        err(u'找不到 carebot_nav_kit 包，请先 source devel/setup.bash')
        return 2

    mp = args.map or os.path.join(pkg, 'maps', 'sim_map.yaml')
    if not os.path.isfile(mp):
        err(u'找不到地图：%s' % mp)
        # ⚠️ pkg 可能来自 argparse（Py2 下是字节串），
        #    插进含中文的 unicode 格式串会崩。
        say(u'  先生成：python2 %s/scripts/make_sim_map.py'
            % to_text(pkg))
        return 2

    tester = NavTester(pkg, mp, goal_timeout=args.timeout)
    try:
        rc = tester.run()
    finally:
        if not args.keep:
            say(u'\n清理后台进程…')
            cleanup()
        else:
            say(u'\n[--keep] 保留进程。清理：pkill -f rosmaster; pkill -f roslaunch')
    return rc


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        cleanup()
        sys.exit(130)
