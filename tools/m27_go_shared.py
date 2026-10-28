# -*- coding: utf-8 -*-
"""
m27_go_shared.py —— 用「其他组的航点」走 A -> B -> C -> D

## 航点来源

`/home/ub/laoyou/talos_scripts/waypoints.yaml`（只读参考，未修改），
注释写明是"按地图估算 + 小键盘实测"的：

    start: {0.00,  0.00}
    A:     {1.17, -1.26, yaw 10}
    D_door:{1.77,  0.09, yaw 5}   D:{2.53,  0.00, yaw 5}
    C_door:{1.30, -1.07, yaw 5}   C:{2.69, -0.92, yaw 0}
    B_door:{1.88, -1.93, yaw -15} B:{2.55, -2.01, yaw 0}

## 他们的核心技巧：门口点（_door）

原注释：
    「门口点（_door）在中间墙和隔板末端之间的"走廊"正中，正对房间；
      进/出房间都先经过它，笔直进出不蹭墙」

也就是说 **不要直接朝房间中心做直线规划**，而是先到门口点，再笔直进房间。
这正好解决我们之前的"目标不可达 -> 恢复行为乱走 -> 蹭墙"问题。

所以本脚本的路线是：
    A -> B_door -> B -> B_door -> C_door -> C -> C_door -> D_door -> D
（出房间也要先回到门口点，避免在房间里掉头扫到隔断）
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

# (名称, x, y, yaw_deg)
SEQ = [
    ('A',        1.17, -1.26,  10),
    ('B_door',   1.88, -1.93, -15),
    ('B',        2.55, -2.01,   0),
    ('B_door',   1.88, -1.93, -15),
    ('C_door',   1.30, -1.07,   5),
    ('C',        2.69, -0.92,   0),
    ('C_door',   1.30, -1.07,   5),
    ('D_door',   1.77,  0.09,   5),
    ('D',        2.53,  0.00,   5),
]

MIN_CLEAR = 0.22        # m  前方小于此值中止（车半径 0.18）
GOAL_TIMEOUT = 120.0    # s


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


def make_goal(x, y, yaw_deg):
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = 'map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = x
    g.target_pose.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw_deg))
    g.target_pose.pose.orientation.x = q[0]
    g.target_pose.pose.orientation.y = q[1]
    g.target_pose.pose.orientation.z = q[2]
    g.target_pose.pose.orientation.w = q[3]
    return g


def main():
    rospy.init_node('m27_go_shared', anonymous=True)
    guard = Guard()
    lst = tf.TransformListener()
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    print('')
    print('=' * 68)
    print('  A -> B -> C -> D（其他组的航点 + 门口点策略）')
    print('=' * 68)
    for i, (nm, x, y, yw) in enumerate(SEQ):
        print('    %d) %-7s (%6.2f, %6.2f) yaw %+4d' % (i + 1, nm, x, y, yw))
    print('  安全阈值: 前方 < %.2f m 中止' % MIN_CLEAR)

    if not ac.wait_for_server(rospy.Duration(25.0)):
        print('  [FAIL] move_base action server 不可用')
        return 1
    print('  action server OK')
    time.sleep(1.0)

    try:
        lst.waitForTransform('/map', '/base_footprint',
                             rospy.Time(0), rospy.Duration(3.0))
        (tr, _r) = lst.lookupTransform('/map', '/base_footprint',
                                       rospy.Time(0))
        print('  出发位姿: (%.3f, %.3f)' % (tr[0], tr[1]))
    except Exception:
        pass

    results = []
    for i, (nm, x, y, yw) in enumerate(SEQ):
        guard.reset()
        print('')
        print('---- [%d/%d] %s -> (%.2f, %.2f) yaw %d ----'
              % (i + 1, len(SEQ), nm, x, y, yw))
        ac.send_goal(make_goal(x, y, yw))
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
            (tr, _r) = lst.lookupTransform('/map', '/base_footprint',
                                           rospy.Time(0))
            err = math.hypot(tr[0] - x, tr[1] - y)
            print('  结果: %-11s 用时 %5.1f s  到达 (%.2f, %.2f)  误差 %.3f m'
                  % (label, dt, tr[0], tr[1], err))
        except Exception:
            print('  结果: %-11s 用时 %.1f s' % (label, dt))
        results.append((nm, label, dt))

    print('')
    print('=' * 68)
    for nm, label, dt in results:
        print('    %-7s %-11s %5.1f s' % (nm, label, dt))
    ok = sum(1 for _n, l, _d in results if l == 'SUCCEEDED')
    print('  成功 %d / %d' % (ok, len(results)))
    print('=' * 68)
    return 0


if __name__ == '__main__':
    sys.exit(main())
