# -*- coding: utf-8 -*-
"""m35_mission_v4.py —— V4 赛题流程（代码版，未做实车验证）

## 流程（按赛题）

    ① 从起点出发 -> 去「起点对面的小房间」（二维码区，qr_room）
    ② 在原地**旋转扫描**（8 × 45°），等待二维码结果
    ③ 由二维码确认 B/C/D 三个房间的**巡视顺序**
    ④ 移动到「A 房间中间」（A_mid）
    ⑤ 到 A 点核验区（A）
    ⑥ 按二维码给的顺序，逐房间「门口点 -> 房间中心」
    ⑦ 每个房间中心的**到达朝向 = 朝下一个目标**（省掉原地掉头，V3 的 68 s 就是掉头掉的）

## 二维码结果怎么拿

沿用 `carebot_eldercare/mission_manager.py` 定的接口：

    /qr/room_order   std_msgs/Int16MultiArray

取值 1/2/3 代表房间，顺序即巡视顺序（例：[3,1,2] = D -> B -> C）。
也可以直接命令行覆盖（便于没二维码时演练）：

    python _tools/real_test/m35_mission_v4.py --order CBD

拿不到二维码时**不硬等**：转满 8 个方向（约 12 s）后回落默认 ``BCD`` 并大声打印警告。

## 安全

与 m29/m30/m33 同一套：用**原始 /scan**，车体自遮挡按固定角度带剔除，
前方 ±20° 锥内 < 0.40 m 立即取消并停车；原地旋转另加"全向 < 0.30 m 停转"。

## 用法

    python _tools/real_test/m35_mission_v4.py --dry-run        # 只打印计划，不动车
    python _tools/real_test/m35_mission_v4.py --order CBD      # 指定顺序
    python _tools/real_test/m35_mission_v4.py                  # 正常跑（从二维码拿顺序）
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

try:
    from std_msgs.msg import Int16MultiArray
except ImportError:                     # pragma: no cover
    Int16MultiArray = None

# ===========================================================================
#  航点（共享地图 talos_map_1016_0808 坐标系）
#  ★ 带星号的点是按地图估的，第一次实车请用 capture_pose 重标
# ===========================================================================
WP = {
    'start':   (0.00,  0.00,   0),
    'qr_room': (0.70, -2.00,   0),   # ★ 起点对面的小房间（A 房间远端），离墙 0.53 m
    'A_mid':   (0.78, -0.96,   0),   # ★ A 房间中间，离墙 0.60 m
    'A':       (1.17, -1.26,  10),   # A 点核验区（别人的航点）
    'B_door':  (1.88, -1.93,  -7),
    'B':       (2.55, -2.01,   0),
    'C_door':  (1.90, -0.95,   0),
    'C':       (2.69, -0.92,   0),
    'D_door':  (1.77,  0.09,  -7),
    'D':       (2.53,  0.00,   5),
}

# 房间号 -> (门口点, 房间中心)；二维码里的 1/2/3 就是按这个顺序映射
ROOMS = {1: ('B_door', 'B'), 2: ('C_door', 'C'), 3: ('D_door', 'D')}
DEFAULT_ORDER = [1, 2, 3]

# ---- 安全参数（与 m29/m30/m33 一致）----
FRONT_CONE = math.radians(20)
FRONT_STOP = 0.40
SPIN_STOP = 0.30
SELF_MIN = 0.36
SELF_BANDS = ((15, 35), (145, 215))
GOAL_TIMEOUT = 120.0
SPIN_STEP_DEG = 45.0
SPIN_SPEED = 0.6            # rad/s
SPIN_TOL = 0.06             # rad
QR_WAIT = 3.0               # 每个朝向上等二维码的时间 s


def is_self(deg):
    d = abs(((deg + 180.0) % 360.0) - 180.0)
    return any(lo <= d <= hi for lo, hi in SELF_BANDS)


class Guard(object):
    """前向 / 全向净空，原始 /scan，剔车体自遮挡。"""

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


class QrWait(object):
    """等 /qr/room_order（Int16MultiArray，1/2/3 的排列）。"""

    def __init__(self):
        self.order = None
        if Int16MultiArray is not None:
            rospy.Subscriber('/qr/room_order', Int16MultiArray, self.cb, queue_size=1)

    def cb(self, m):
        vals = [int(v) for v in m.data]
        if sorted(vals) == [1, 2, 3]:
            self.order = vals
            rospy.logwarn('从 /qr/room_order 收到顺序: %s' % vals)


def yaw_to(x0, y0, x1, y1):
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


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


def build_legs(order):
    """把「顺序」展开成一条条腿。房间中心的到达朝向 = 朝下一个目标。"""
    legs = [('qr_room', '① 起点对面的小房间（二维码区）')]
    legs.append(('A_mid', '④ A 房间中间'))
    legs.append(('A', '⑤ A 点核验区'))
    for k, rid in enumerate(order):
        door, center = ROOMS[rid]
        tag = chr(ord('B') + (rid - 1))
        legs.append((door, '⑥ 第%d站 %s 门口' % (k + 1, tag)))
        legs.append((center, '⑥ 第%d站 %s 房间中心' % (k + 1, tag)))
    # 逐腿算 yaw：默认用航点自带 yaw，房间中心改成朝下一条腿
    out = []
    for i, (name, label) in enumerate(legs):
        x, y, yaw = WP[name]
        if name in ('B', 'C', 'D') and i + 1 < len(legs):
            nx, ny, _ = WP[legs[i + 1][0]]
            yaw = yaw_to(x, y, nx, ny)
        out.append((name, x, y, round(yaw, 1), label))
    return out


def spin_scan(guard, stop_pub, lst, qr):
    """原地按 SPIN_STEP_DEG 转一圈找二维码；找到就提前退出。"""
    print('  ② 原地旋转扫描二维码（每步 %.0f°，最多一圈）' % SPIN_STEP_DEG)
    start = pose_of(lst)
    if start is None:
        print('     [warn] 拿不到位姿，跳过旋转扫描')
        return qr.order
    for k in range(int(round(360.0 / SPIN_STEP_DEG))):
        if qr.order is not None:
            break
        cur = pose_of(lst)
        if cur is None:
            break
        target = math.radians(cur[2] + SPIN_STEP_DEG)
        t0 = time.time()
        while not rospy.is_shutdown():
            if guard.min_all < SPIN_STOP:
                print('     ⛔ 全向净空 %.2f m < %.2f m，停止旋转' % (guard.min_all, SPIN_STOP))
                stop_pub.publish(Twist())
                return qr.order
            cur = pose_of(lst)
            if cur is None:
                break
            err = math.atan2(math.sin(target - math.radians(cur[2])),
                             math.cos(target - math.radians(cur[2])))
            if abs(err) < SPIN_TOL or time.time() - t0 > 6.0:
                break
            tw = Twist()
            tw.angular.z = SPIN_SPEED if err > 0 else -SPIN_SPEED
            stop_pub.publish(tw)
            time.sleep(0.05)
        stop_pub.publish(Twist())
        time.sleep(0.4)
        # 停在与这一步对齐的朝向上，扫一会儿
        t1 = time.time()
        while time.time() - t1 < QR_WAIT and qr.order is None and not rospy.is_shutdown():
            time.sleep(0.1)
        cur = pose_of(lst)
        print('     第 %d 个朝向: yaw %s  二维码=%s'
              % (k + 1, ('%.0f°' % cur[2]) if cur else '?', qr.order))
    stop_pub.publish(Twist())
    return qr.order


def main():
    dry = '--dry-run' in sys.argv
    order_override = None
    if '--order' in sys.argv:
        s = sys.argv[sys.argv.index('--order') + 1].upper()
        m = {'B': 1, 'C': 2, 'D': 3}
        order_override = [m[c] for c in s]
        assert sorted(order_override) == [1, 2, 3], '--order 必须是 BCD 的排列'

    rospy.init_node('mission_v4', anonymous=not dry)
    lst = tf.TransformListener()
    guard = Guard()
    qr = QrWait()
    stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    print('')
    print('=' * 72)
    print('  V4 赛题流程')
    print('=' * 72)
    print('  ① 起点对面的小房间 (%.2f, %.2f)   ② 旋转扫二维码' % WP['qr_room'][:2])
    print('  ③ 由二维码定 B/C/D 顺序          ④ A 房间中间 (%.2f, %.2f)' % WP['A_mid'][:2])
    print('  ⑤ A 点核验区                    ⑥ 按顺序逐房间')
    if order_override:
        print('  顺序: 命令行指定 %s' % ''.join(chr(ord('A') + v) for v in order_override))
    else:
        print('  顺序: 等 /qr/room_order，%.0f s 内收不到就回落 BCD' % (QR_WAIT * 8))

    if dry:
        print('')
        print('  [dry-run] 计划（不动车）:')
        for name, x, y, yaw, label in build_legs(order_override or DEFAULT_ORDER):
            print('    %-8s (%5.2f, %5.2f) yaw %+6.1f   %s' % (name, x, y, yaw, label))
        print('')
        print('  合计 %d 腿' % len(build_legs(order_override or DEFAULT_ORDER)))
        return 0

    if not ac.wait_for_server(rospy.Duration(25.0)):
        print('  [FAIL] move_base 不可用')
        return 1
    print('  出发位姿: %s' % (pose_of(lst),))

    results = []

    def goto(name, x, y, yaw, label):
        print('')
        print('---- %s  %s -> (%.2f, %.2f, %.0f°) ----' % (label, name, x, y, yaw))
        t0 = time.time()
        ac.send_goal(make_goal(x, y, yaw))
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
            print('   %5.1fs %-9s 前向 %.2f m  全向 %.2f m' % (el, 'RUNNING',
                                                              guard.min_front, guard.min_all))
            time.sleep(1.0)
        if state == 'ABORT_SAFETY':
            print('   ⛔ %s —— 已取消并停车' % note)
        stop_pub.publish(Twist())
        p = pose_of(lst)
        dt = time.time() - t0
        err = math.hypot(p[0] - x, p[1] - y) if p else float('nan')
        print('   结果 %-12s 用时 %5.1f s 到达 (%s) 误差 %.3f m'
              % (state, dt, ('%.2f, %.2f' % (p[0], p[1])) if p else '?', err))
        results.append((name, state, dt, err))
        return state == 'SUCCEEDED'

    # ① 到二维码区
    _, x, y, yaw, _ = build_legs(DEFAULT_ORDER)[0]
    if not goto('qr_room', x, y, yaw, '① 前往二维码区'):
        print('  [warn] 没能到达二维码区，仍尝试原地扫描')
    time.sleep(0.5)

    # ② 旋转扫描
    got = spin_scan(guard, stop_pub, lst, qr)

    # ③ 定顺序
    if order_override:
        order = order_override
        src = '命令行'
    elif got:
        order = got
        src = '二维码'
    else:
        order = list(DEFAULT_ORDER)
        src = '★回落默认（没收到二维码）'
    print('')
    print('  ③ 巡视顺序: %s   （来源: %s）'
          % (' -> '.join(chr(ord('A') + v) for v in order), src))

    # ④⑤⑥ 按顺序走
    legs = build_legs(order)
    for name, xx, yy, yyaw, label in legs[1:]:
        goto(name, xx, yy, yyaw, label)

    print('')
    print('=' * 72)
    print('  汇总')
    print('=' * 72)
    for name, state, dt, err in results:
        print('    %-8s %-12s %6.1f s  误差 %.3f m' % (name, state, dt, err))
    ok = sum(1 for r in results if r[1] == 'SUCCEEDED')
    print('  成功 %d / %d   总用时 %.1f s' % (ok, len(results), sum(r[2] for r in results)))
    stop_pub.publish(Twist())
    return 0 if ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
