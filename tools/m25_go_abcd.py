# -*- coding: utf-8 -*-
"""
m25_go_abcd.py —— 按 A -> B -> C -> D 走一遍（目标点来自官方平面图 + 余量最大化）

## 目标点（由 room_centers.py 的邻域余量最大化得到）

    A 客厅  (-1.837, 0.138)   余量 0.62 m
    B 书房  (-0.812, 1.288)   余量 0.93 m
    C 厨房  (-1.387, 1.463)   余量 0.90 m
    D 卧室  (-2.112, 2.038)   余量 0.53 m

余量 = 该点到最近障碍的距离，全部 > 车半径 0.28 m，停车安全。

## 与 m9 的区别

  · **不再设置初始位姿** —— 已由 m24_slam_to_nav.sh 用 gmapping 真值设好，
    重复设置反而可能把 AMCL 推偏。
  · 目标是官方平面图推出的房间中心，不是四象限几何中心。

## 安全

  · 订阅 /scan_filtered，前方 < 0.28 m 立刻取消目标并停车
  · 每个目标超时 150 s，失败不重试（先看数据）
  · 全程打印实际位姿，方便判断有没有跑偏
"""
from __future__ import print_function

import math
import sys
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

WAYPOINTS = [
    ('A_hall',  -1.837,  0.138),
    ('B_study', -0.812,  1.288),
    ('C_kitchen', -1.387, 1.463),
    ('D_bedroom', -2.112, 2.038),
]

MIN_CLEAR = 0.28
GOAL_TIMEOUT = 150.0


class Guard(object):
    def __init__(self):
        self.min_front = None
        self.trip = False
        rospy.Subscriber('/scan_filtered', LaserScan, self._cb, queue_size=3)

    def _cb(self, m):
        best = None
        for i, v in enumerate(m.ranges):
            if v != v or v == float('inf') or v < 0.05:
                continue
            a = m.angle_min + i * m.angle_increment
            if abs(a) > math.radians(35.0):
                continue
            if best is None or v < best:
                best = v
        self.min_front = best
        if best is not None and best < MIN_CLEAR:
            self.trip = True

    def reset(self):
        self.trip = False


def make_goal(x, y):
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = 'map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = x
    g.target_pose.pose.position.y = y
    g.target_pose.pose.orientation.w = 1.0
    return g


def main():
    rospy.init_node('m25_go_abcd', anonymous=True)
    guard = Guard()
    lst = tf.TransformListener()
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    print('')
    print('=' * 66)
    print('  A -> B -> C -> D 导航测试（官方平面图房间中心）')
    print('=' * 66)
    for i, (nm, x, y) in enumerate(WAYPOINTS):
        print('    %d) %-11s (%6.3f, %6.3f)' % (i + 1, nm, x, y))
    print('  安全阈值: 前方 < %.2f m 中止' % MIN_CLEAR)

    if not ac.wait_for_server(rospy.Duration(25.0)):
        print('  [FAIL] move_base action server 不可用')
        return 1
    print('  action server OK')
    time.sleep(1.0)

    try:
        lst.waitForTransform('/map', '/base_footprint',
                             rospy.Time(0), rospy.Duration(3.0))
        (tr, rot) = lst.lookupTransform('/map', '/base_footprint',
                                        rospy.Time(0))
        print('  出发位姿: (%.3f, %.3f)' % (tr[0], tr[1]))
    except Exception:
        pass

    results = []
    for i, (nm, x, y) in enumerate(WAYPOINTS):
        guard.reset()
        print('')
        print('---- [%d/%d] %s -> (%.3f, %.3f) ----'
              % (i + 1, len(WAYPOINTS), nm, x, y))
        ac.send_goal(make_goal(x, y))
        t0 = time.time()
        state = None
        while time.time() - t0 < GOAL_TIMEOUT and not rospy.is_shutdown():
            if guard.trip:
                print('  ★ 前方 %.2f m < %.2f m —— 取消并停车'
                      % (guard.min_front, MIN_CLEAR))
                ac.cancel_goal()
                state = 'ABORT_SAFETY'
                break
            st = ac.get_state()
            if st in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                      GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                state = st
                break
            time.sleep(0.3)
        if state is None:
            print('  [超时] %.0f s' % GOAL_TIMEOUT)
            ac.cancel_goal()
            state = 'TIMEOUT'

        names = {GoalStatus.SUCCEEDED: 'SUCCEEDED',
                 GoalStatus.ABORTED: 'ABORTED',
                 GoalStatus.REJECTED: 'REJECTED',
                 GoalStatus.PREEMPTED: 'PREEMPTED'}
        label = names.get(state, str(state))
        dt = time.time() - t0
        try:
            lst.waitForTransform('/map', '/base_footprint',
                                 rospy.Time(0), rospy.Duration(3.0))
            (tr, rot) = lst.lookupTransform('/map', '/base_footprint',
                                            rospy.Time(0))
            err = math.hypot(tr[0] - x, tr[1] - y)
            print('  结果: %-11s 用时 %5.1f s  到达 (%.3f, %.3f)  误差 %.3f m'
                  % (label, dt, tr[0], tr[1], err))
        except Exception:
            print('  结果: %-11s 用时 %.1f s' % (label, dt))
        results.append((nm, label, dt))
        time.sleep(2.0)

    print('')
    print('=' * 66)
    print('  汇总')
    for nm, label, dt in results:
        print('    %-11s %-11s %5.1f s' % (nm, label, dt))
    ok = sum(1 for _n, l, _d in results if l == 'SUCCEEDED')
    print('  成功 %d / %d' % (ok, len(results)))
    print('=' * 66)
    return 0


if __name__ == '__main__':
    sys.exit(main())
