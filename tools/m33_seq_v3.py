# -*- coding: utf-8 -*-
"""m33_seq_v3.py —— V3 路线：8 段，去掉回程门口点，C_door 移到走廊侧

    python _tools/real_test/m30_full_sequence.py                 # 默认 10 s
    python _tools/real_test/m30_full_sequence.py --wait 10

安全：与 m29 同一套——用**原始 /scan**（车体自遮挡按固定角度带剔除），
前方 ±20° 锥内 < 0.40 m 立即取消并停车。
"""
from __future__ import print_function

import math
import sys
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

FRONT_CONE = math.radians(20)
FRONT_STOP = 0.40
SELF_MIN = 0.36
SELF_BANDS = ((15, 35), (145, 215))     # 车体自遮挡（激光系，实测 0.10~0.45 m）
GOAL_TIMEOUT = 120.0

START = ('start', 0.00, 0.00, 0)
# V3：不再"进房-回门口-再进另一房"，出房直接去下一个门口点；
#     C_door 从门口的 (1.30,-1.07) 挪到走廊侧的 (1.90,-0.95)，
#     省掉两次穿过中间墙门洞的折返（直线距离 11.23 -> 7.54 m）
SEQ = [
    ('A',      1.17, -1.26,  10),
    ('B_door', 1.88, -1.93,  -7),
    ('B',      2.55, -2.01,   0),
    ('C_door', 1.90, -0.95,   0),
    ('C',      2.69, -0.92,   0),
    ('D_door', 1.77,  0.09,   5),
    ('D',      2.53,  0.00,   5),
]


def is_self(deg):
    d = abs(((deg + 180.0) % 360.0) - 180.0)
    return any(lo <= d <= hi for lo, hi in SELF_BANDS)


class Guard(object):
    def __init__(self):
        self.min_front = 9.0
        self.min_all = 9.0
        self.front_deg = 0.0
        rospy.Subscriber('/scan', LaserScan, self.cb, queue_size=1)

    def cb(self, s):
        best, allmin = 3.0, 3.0
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
        self.min_front, self.min_all = best, allmin


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


def pose_of(lst):
    try:
        lst.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
        (t, r) = lst.lookupTransform('map', 'base_footprint', rospy.Time(0))
        e = tf.transformations.euler_from_quaternion(r)
        return t[0], t[1], math.degrees(e[2])
    except Exception:
        return None


def main():
    wait_s = 0.0
    if '--wait' in sys.argv:
        wait_s = float(sys.argv[sys.argv.index('--wait') + 1])

    rospy.init_node('m33_seq', anonymous=True)
    guard = Guard()
    lst = tf.TransformListener()
    stop = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    legs = [START] + SEQ
    print('')
    print('=' * 72)
    print('  先回起点 (0.00, 0.00)，停 %.0f s，再走 A→B→C→D' % wait_s)
    print('=' * 72)
    for i, (nm, x, y, yw) in enumerate(legs):
        print('    %2d) %-7s (%6.2f, %6.2f) yaw %+4d%s'
              % (i + 1, nm, x, y, yw, '   <-- 起点，之后停 %.0f s' % wait_s if i == 0 else ''))

    if not ac.wait_for_server(rospy.Duration(25.0)):
        print('  [FAIL] move_base 不可用')
        return 1
    print('  出发位姿: %s' % (pose_of(lst),))

    results = []
    for i, (nm, x, y, yw) in enumerate(legs):
        print('')
        print('---- [%d/%d] %s -> (%.2f, %.2f, %d°) ----' % (i + 1, len(legs), nm, x, y, yw))
        t0 = time.time()
        ac.send_goal(make_goal(x, y, yw))
        state, note = None, ''
        while not rospy.is_shutdown():
            el = time.time() - t0
            if guard.min_front < FRONT_STOP:
                note = '前方 %.0f° %.2f m < %.2f m' % (guard.front_deg, guard.min_front, FRONT_STOP)
                ac.cancel_goal()
                state = 'ABORT_SAFETY'
                break
            st = ac.get_state()
            if st == GoalStatus.SUCCEEDED:
                state = 'SUCCEEDED'
                break
            if st in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                state = {GoalStatus.ABORTED: 'ABORTED', GoalStatus.REJECTED: 'REJECTED',
                         GoalStatus.PREEMPTED: 'PREEMPTED'}[st]
                break
            if el > GOAL_TIMEOUT:
                ac.cancel_goal()
                state = 'TIMEOUT'
                break
            print('   %5.1fs  %-9s 前向 %.2f m  全向 %.2f m'
                  % (el, 'RUNNING', guard.min_front, guard.min_all))
            time.sleep(1.0)
        if state == 'ABORT_SAFETY':
            print('   ⛔ %s —— 已取消并停车' % note)
        stop.publish(Twist())
        time.sleep(0.6)
        p = pose_of(lst)
        dt = time.time() - t0
        err = math.hypot(p[0] - x, p[1] - y) if p else float('nan')
        print('   结果 %-12s 用时 %5.1f s  到达 (%s)  误差 %.3f m'
              % (state, dt, ('%.2f, %.2f' % (p[0], p[1])) if p else '?', err))
        results.append((nm, state, dt, p, err))

        if i == 0:
            print('   —— 到起点，按要求停 %.0f s ——' % wait_s)
            time.sleep(wait_s)

    print('')
    print('=' * 72)
    print('  汇总')
    print('=' * 72)
    ok = 0
    for nm, state, dt, p, err in results:
        print('    %-7s %-12s %6.1f s   误差 %.3f m' % (nm, state, dt, err))
        if state == 'SUCCEEDED':
            ok += 1
    print('  成功 %d / %d' % (ok, len(results)))
    stop.publish(Twist())
    return 0 if ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
