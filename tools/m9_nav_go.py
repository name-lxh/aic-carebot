# -*- coding: utf-8 -*-
"""
m9_nav_go.py —— 设初始位姿 + 顺时针依次导航四个房间

## 顺序（顺时针，从左上开始）

    红 左上 (x-,y+)  ->  黄 右上 (x+,y+)  ->  绿 右下 (x+,y-)  ->  蓝 左下 (x-,y-)

坐标来自 `_tools/rooms_quad.py`（在干净地图 arena_clean 上算的，
每个点都要求到障碍的安全余量 >= 0.38 m）。

## 安全

  · 全程订阅 `/scan_filtered`，**任一时刻前方 < 0.28 m 就取消当前目标并停车**
  · 每个目标有超时（默认 90 s）
  · 每个目标之间停 2 s 再发下一个
  · 失败的目标不重试（先看一遍数据，别硬来）

## 注意

初始位姿是个**猜测**（机器人被搬到场地中间，朝向未知）。
AMCL 会用激光去修，但如果初始误差太大，第一段可能会走歪 ——
所以第一个目标特意选了一个较近、空间较大的点。
"""
from __future__ import print_function

import math
import sys
import threading
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

# ---- 顺时针四个点（来自 rooms_quad.py）------------------------------------
WAYPOINTS = [
    ('NW_red',   -1.56,  0.79),
    ('NE_yellow', -0.14,  0.71),
    ('SE_green',  0.04, -0.84),
    ('SW_blue',  -1.54, -0.89),
]

# ---- 初始位姿：机器人在场地中心附近，朝向未知先按 0 -----------------------
# ⚠️⚠️ 这个位姿是用 localize.py 做**全局穷举匹配**得到的，不是猜的。
#     之前按 "机器人在中间、朝向 0" 猜过一次，位置对但**朝向差 160°**，
#     导致 move_base 把车往反方向带（目标在西北，车却往南走了 1.4 m 然后 ABORTED）。
#     实测最优解得分 3.78 格 = 0.094 m（扫描点平均离墙 9.4 cm），
#     次优解 6.70（差 77%），唯一性很好。
# 机器人现在停在**起点（A 房间右下角支路的缺口）**，
# 位姿来自 localize.py 全局匹配 + AMCL 收敛确认。
# 车被手动移动过 -> 重跑 localize.py 全局匹配得到：
#   最佳 (0.340,-1.100,125°) 得分 0.162m，次优差 10%，
#   且 (0.4,-0.9~-1.2) 附近有一簇 yaw≈125° 的一致解。
INIT_X, INIT_Y, INIT_YAW_DEG = 0.340, -1.100, 125.0

MIN_CLEAR = 0.28          # m  前方小于这个就中止（车半径 0.25）
GOAL_TIMEOUT = 150.0      # s  实测 90 s 不够（之前 90 s 只走到一半）


class Guard(object):
    """实时安全护栏：前方太近就置位。"""

    def __init__(self):
        self.min_front = None
        self.trip = False
        self._lock = threading.Lock()
        rospy.Subscriber('/scan_filtered', LaserScan, self._cb, queue_size=3)

    def _cb(self, m):
        best = None
        n = len(m.ranges)
        for i, v in enumerate(m.ranges):
            if v != v or v == float('inf') or v < 0.05:
                continue
            a = m.angle_min + i * m.angle_increment
            # 前方 ±35°
            if abs(a) > math.radians(35.0):
                continue
            if best is None or v < best:
                best = v
        with self._lock:
            self.min_front = best
            if best is not None and best < MIN_CLEAR:
                self.trip = True

    def read(self):
        with self._lock:
            return self.min_front, self.trip

    def reset(self):
        with self._lock:
            self.trip = False


def set_initial_pose(x, y, yaw_deg):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                          queue_size=1, latch=True)
    time.sleep(1.0)
    p = PoseWithCovarianceStamped()
    p.header.frame_id = 'map'
    p.header.stamp = rospy.Time.now()
    p.pose.pose.position.x = x
    p.pose.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(
        0, 0, math.radians(yaw_deg))
    p.pose.pose.orientation.x = q[0]
    p.pose.pose.orientation.y = q[1]
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]
    # 给一个较大的初始协方差，让 AMCL 敢修
    cov = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
    p.pose.covariance = cov
    for _ in range(6):
        pub.publish(p)
        time.sleep(0.3)
    print('  已发布 /initialpose: (%.2f, %.2f) yaw=%.0f°'
          % (x, y, yaw_deg))


def make_goal(x, y, yaw_deg=0.0):
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
    rospy.init_node('m9_nav_go', anonymous=True)
    guard = Guard()
    lst = tf.TransformListener()
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    print('')
    print('=' * 68)
    print('  导航验证：顺时针依次访问四个房间')
    print('=' * 68)
    print('  目标顺序:')
    for i, (nm, x, y) in enumerate(WAYPOINTS):
        print('    %d) %-10s (%.2f, %.2f)' % (i + 1, nm, x, y))
    print('  安全阈值: 前方 < %.2f m 中止' % MIN_CLEAR)
    print('')

    print('  等待 move_base action server ...')
    if not ac.wait_for_server(rospy.Duration(20.0)):
        print('  [FAIL] move_base action server 不可用')
        return 1
    print('  OK')

    time.sleep(2.0)
    print('')
    print('---- 设定初始位姿 ----')
    set_initial_pose(INIT_X, INIT_Y, INIT_YAW_DEG)
    print('  等 AMCL 收敛（看 /amcl_pose）...')
    try:
        ap = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped,
                                    timeout=15.0)
        pp = ap.pose.pose.position
        print('  AMCL 当前认为: (%.2f, %.2f)' % (pp.x, pp.y))
    except Exception as e:
        print('  [!] 拿不到 /amcl_pose: %r' % e)

    results = []
    for i, (nm, x, y) in enumerate(WAYPOINTS):
        guard.reset()
        print('')
        print('---- [%d/%d] %s  ->  (%.2f, %.2f) ----'
              % (i + 1, len(WAYPOINTS), nm, x, y))
        try:
            lst.waitForTransform('/map', '/base_footprint',
                                 rospy.Time(0), rospy.Duration(3.0))
            (tr, _r) = lst.lookupTransform('/map', '/base_footprint',
                                           rospy.Time(0))
            print('  出发位姿: (%.2f, %.2f)' % (tr[0], tr[1]))
        except Exception:
            pass

        g = make_goal(x, y, 0.0)
        ac.send_goal(g)
        t0 = time.time()
        state = None
        while time.time() - t0 < GOAL_TIMEOUT and not rospy.is_shutdown():
            mf, trip = guard.read()
            if trip:
                print('  ★ 前方 %.2f m < %.2f m —— 取消目标并停车！'
                      % (mf, MIN_CLEAR))
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
            print('  [超时] %.0f s 未完成，取消' % GOAL_TIMEOUT)
            ac.cancel_goal()
            state = 'TIMEOUT'

        names = {GoalStatus.SUCCEEDED: 'SUCCEEDED',
                 GoalStatus.ABORTED: 'ABORTED',
                 GoalStatus.REJECTED: 'REJECTED',
                 GoalStatus.PREEMPTED: 'PREEMPTED'}
        label = names.get(state, str(state))
        dt = time.time() - t0
        print('  结果: %s   用时 %.1f s' % (label, dt))
        try:
            lst.waitForTransform('/map', '/base_footprint',
                                 rospy.Time(0), rospy.Duration(3.0))
            (tr, rot) = lst.lookupTransform('/map', '/base_footprint',
                                            rospy.Time(0))
            print('  到达位姿: (%.2f, %.2f)' % (tr[0], tr[1]))
        except Exception:
            pass
        results.append((nm, label, dt))
        time.sleep(2.0)

    print('')
    print('=' * 68)
    print('  汇总')
    for nm, label, dt in results:
        print('    %-10s %-12s %5.1f s' % (nm, label, dt))
    ok = sum(1 for _n, l, _d in results if l == 'SUCCEEDED')
    print('  成功 %d / %d' % (ok, len(results)))
    print('=' * 68)
    return 0


if __name__ == '__main__':
    sys.exit(main())
