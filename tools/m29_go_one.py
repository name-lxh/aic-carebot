# -*- coding: utf-8 -*-
"""m29_go_one.py —— 发一个目标点，带实时安全中止（只走一小段用）

## 为什么要单独一个

2026-10-20 第六轮 9 个航点全灭，真因是**起点位姿落在墙里**（见 docs）。
修好位姿后先用一个**近距离、同房间**的目标验证整条链路，
不要一上来就发一串航点——那样失败了又分不清是哪一段的问题。

## 安全

· 用 **/scan 原始**做前向净空判断。`/scan_filtered` 下限 0.50 m，
  近距离（恰恰最危险的时候）读数被滤掉了，不能用来做安全判断。
· 车体自遮挡点（0.10~0.31 m，集中在 ±20°/±150~210°）用 r>0.32 挡掉。
· 前方 ±50° 内最近点 < 0.35 m → 立即取消目标并停车。

## 用法

    python _tools/real_test/m29_go_one.py X Y YAW_DEG [超时秒]
    python _tools/real_test/m29_go_one.py 0.80 -1.29 0 60
"""
from __future__ import print_function

import math
import sys
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler

FRONT_CONE = math.radians(20)
FRONT_STOP = 0.40
SELF_MIN = 0.36
# 车体自遮挡的固定角度带（激光系，实测 0.10~0.45 m 都出现在这些角度）：
#   ±(15~35)  前方偏左的支架    ±(145~215) 背后车体
SELF_BANDS = ((15, 35), (145, 215))


def is_self(deg):
    d = abs(((deg + 180.0) % 360.0) - 180.0)
    for lo, hi in SELF_BANDS:
        if lo <= d <= hi:
            return True
    return False


class Guard(object):
    def __init__(self):
        self.min_front = 99.0
        self.min_all = 99.0
        self.t = 0.0
        self.front_deg = 0.0
        rospy.Subscriber('/scan', LaserScan, self.cb, queue_size=1)

    def cb(self, s):
        best = 3.0          # 前方锥内没有点 = 3 m 内通畅
        allmin = 3.0
        for i, v in enumerate(s.ranges):
            if not (v == v) or v < SELF_MIN or v > 3.0:
                continue
            a = math.atan2(math.sin(s.angle_min + i * s.angle_increment),
                           math.cos(s.angle_min + i * s.angle_increment))
            if is_self(math.degrees(a)) and v < 0.50:
                continue
            if v < allmin:
                allmin = v
            if abs(a) < FRONT_CONE and v < best:
                best = v
                self.front_deg = math.degrees(a)
        self.min_front = best
        self.min_all = allmin
        self.t = time.time()


def main():
    x, y = float(sys.argv[1]), float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3]))
    timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0

    rospy.init_node('go_one', anonymous=True)
    guard = Guard()
    stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print('等 move_base action server ...')
    if not ac.wait_for_server(rospy.Duration(20.0)):
        print('  [FAIL] 没有 move_base')
        return 1

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    print('目标 (%.2f, %.2f, %.0f°)  超时 %.0f s  前向安全阈值 %.2f m'
          % (x, y, math.degrees(yaw), timeout, FRONT_STOP))
    start = time.time()
    ac.send_goal(goal)
    aborted = None
    while not rospy.is_shutdown():
        st = ac.get_state()
        el = time.time() - start
        # 前向净空不足 → 立即取消
        if guard.min_front < FRONT_STOP:
            print('  ⛔ 前方 %.0f° 处净空 %.2f m < %.2f m，立即取消'
                  % (guard.front_deg, guard.min_front, FRONT_STOP))
            aborted = 'SAFETY'
            ac.cancel_goal()
            break
        if st == GoalStatus.SUCCEEDED:
            break
        if st in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED):
            aborted = {GoalStatus.ABORTED: 'ABORTED', GoalStatus.REJECTED: 'REJECTED',
                       GoalStatus.PREEMPTED: 'PREEMPTED'}[st]
            break
        if el > timeout:
            print('  ⏱ 超时 %.0f s，取消' % timeout)
            aborted = 'TIMEOUT'
            ac.cancel_goal()
            break
        print('  %5.1fs  状态=%d  前方净空 %.2f m  全向最近 %.2f m'
              % (el, st, guard.min_front, guard.min_all))
        time.sleep(1.0)

    if aborted is None:
        print('  ✅ SUCCEEDED  用时 %.1f s' % (time.time() - start))
    else:
        print('  ✗ %s  用时 %.1f s' % (aborted, time.time() - start))
    stop_pub.publish(Twist())
    return 0 if aborted is None else 1


if __name__ == '__main__':
    sys.exit(main())
