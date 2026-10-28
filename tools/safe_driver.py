# -*- coding: utf-8 -*-
"""
safe_driver.py —— 真机安全驱动器（带激光急停 + 里程计闭环）

## 为什么不用 nav2/move_base

场地是刚搭的、没有对应地图，AMCL 无图可用；
而且任务是"走到 A/B/C/D 各停 10 秒"，本质是**受控的相对移动**，
不需要全局规划。用里程计 + 激光护栏更直接、更容易验证。

## ★ 安全设计（这台真机的实际情况决定的）

实测这台底盘的两点特性（来自 `base_control/script/stm32.py`）：

  1. **上电不会自己动** —— `trans_x` 只在收到 `/cmd_vel` 时被赋值
  2. **没有 cmd_vel 超时归零** —— 一旦发了速度，它会**一直保持**
     （`last_cmd_vel_time` 被记录但没被用来刹车）

  第 2 点意味着：**任何异常退出都必须显式发 0 速度**，
  否则机器人会一直往前冲。所以本程序：

    · 所有退出路径（正常、异常、Ctrl-C、信号）都走同一个 `stop()`
    · 起一个**独立的高频看门狗线程**持续发 0，主线程发实际速度
      —— 主线程一旦卡死/崩掉，看门狗会在 `WD_TIMEOUT` 内把速度压回 0
    · 激光前向护栏：任何**行进方向**前方 `< FRONT_STOP` 就立即停

## ★ 近处回波的处理（实测踩过）

实测 `/scan` 里 176/978 个点在 0.10~0.15 m，**集中在机器人背后
150°~210°** —— 那是**背后的墙/隔板**，不是自扫。
但 0.30 m 以内仍有零散点（车体支架/地面噪声）。

所以：**只看行进方向 ±FRONT_HALF 的扇区内的点**，
忽略背后和两侧 —— 后退方向的近处回波本来就不该参与前向判断。
另外把 `< 0.10 m` 的点当无效（比任何真实障碍都近，只能是自扫/噪声）。

## 用法（示例）

    python safe_driver.py --status                # 只读：打印位姿和激光摘要
    python safe_driver.py --rotate 90             # 原地左转 90°
    python safe_driver.py --forward 1.0           # 前进 1.0 m
    python safe_driver.py --goto 1.0 0.0 0        # 走到相对 (1.0, 0.0) 朝 0°
    python safe_driver.py --run-sequence seq.json # 跑一串动作（A/B/C/D 用）

⚠️ 这是真机程序：**默认速度很小**（0.15 m/s），且所有动作都有超时上限。
"""

from __future__ import print_function

import argparse
import json
import math
import sys
import threading
import time

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    import codecs
    try:
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout)
    except Exception:
        pass

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ---------------------------------------------------------------- 安全参数
V_LINEAR = 0.15          # m/s   前进速度（保守）
V_ANGULAR = 0.35         # rad/s 转向速度（保守）
FRONT_HALF = 35.0        # deg   行进方向护栏扇区的半宽
FRONT_STOP = 0.45        # m     前方 < 此距离立即停（车体 0.18 + 膨胀 0.22 + 余量）
MIN_VALID = 0.10         # m     比这更近的点视为自扫/噪声，忽略
WD_TIMEOUT = 0.60        # s     看门狗：超过这么久没收到主线程心跳就压 0
CTRL_HZ = 20.0           # 控制频率


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class SafeDriver(object):
    def __init__(self, dry_run=False):
        self.dry = dry_run
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan = None
        self.odom = None
        self.lock = threading.Lock()
        self._last_beat = time.time()
        self._stopped = threading.Event()
        self._front_min = None

        rospy.Subscriber('/scan', LaserScan, self._on_scan, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self._on_odom, queue_size=10)

        # 看门狗：独立线程，主线程若卡死/崩掉，这里负责把速度压回 0
        self._wd = threading.Thread(target=self._watchdog)
        self._wd.daemon = True
        self._wd.start()

        rospy.on_shutdown(self.stop)

    # ------------------------------------------------------------ 回调
    def _on_scan(self, msg):
        with self.lock:
            self.scan = msg

    def _on_odom(self, msg):
        with self.lock:
            self.odom = msg

    def beat(self):
        self._last_beat = time.time()

    # ------------------------------------------------------------ 安全
    def stop(self, reason=''):
        """发 0 速度。**所有退出路径都必须经过这里。**"""
        if self._stopped.is_set():
            return
        self._stopped.set()
        try:
            for _ in range(5):          # 连发几次，确保底盘收到
                self.pub.publish(Twist())
                time.sleep(0.03)
        except Exception:
            pass
        if reason:
            print('  [stop] %s' % reason)

    def _watchdog(self):
        r = rospy.Rate(CTRL_HZ)
        while not rospy.is_shutdown():
            try:
                if time.time() - self._last_beat > WD_TIMEOUT:
                    # 主线程没心跳了 —— 强制压 0
                    self.pub.publish(Twist())
            except Exception:
                pass
            r.sleep()

    # ------------------------------------------------------------ 位姿
    def pose(self):
        with self.lock:
            if self.odom is None:
                return None
            p = self.odom.pose.pose
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return (p.position.x, p.position.y, yaw)

    def front_min(self, heading_rel=0.0):
        """行进方向（相对车头 heading_rel 弧度）±FRONT_HALF 扇区内的最近距离。

        找不到有效点返回 None（表示"前方畅通"）。
        """
        with self.lock:
            s = self.scan
        if s is None:
            return None
        half = math.radians(FRONT_HALF)
        best = None
        n = len(s.ranges)
        for i, v in enumerate(s.ranges):
            if v != v or v == float('inf') or v < MIN_VALID:
                continue
            a = s.angle_min + i * s.angle_increment
            d = abs(norm_angle(a - heading_rel))
            if d > half:
                continue
            if best is None or v < best:
                best = v
        return best

    # ------------------------------------------------------------ 动作
    def rotate(self, delta_deg, timeout=25.0):
        """原地转 delta_deg（正=逆时针/左转）。"""
        p0 = self.pose()
        if p0 is None:
            print('  ✘ 没有 /odom，无法闭环');  return False
        target = p0[2] + math.radians(delta_deg)
        t0 = time.time()
        cmd = Twist()
        cmd.angular.z = V_ANGULAR if delta_deg > 0 else -V_ANGULAR
        r = rospy.Rate(CTRL_HZ)
        while not rospy.is_shutdown():
            self.beat()
            p = self.pose()
            if p is None:
                break
            err = norm_angle(target - p[2])
            if abs(err) < math.radians(3.0):
                break
            if time.time() - t0 > timeout:
                print('  ⚠️ 转向超时（剩余 %.1f°）' % math.degrees(err))
                break
            cmd.angular.z = math.copysign(
                min(V_ANGULAR, abs(err) * 2.0 + 0.10), err)
            if not self.dry:
                self.pub.publish(cmd)
            r.sleep()
        self.stop()
        p = self.pose()
        if p:
            print('  转向完成：yaw %.1f° -> %.1f°'
                  % (math.degrees(p0[2]), math.degrees(p[2])))
        return True

    def forward(self, dist_m, timeout=30.0):
        """沿当前车头方向前进 dist_m（负值=后退）。

        ⚠️ 只有**前进**时才启用前向激光护栏；后退不启用
        （因为背后 0.10 m 就是墙，那会立刻拦停）。
        后退时改用"更短的距离 + 更低速度"来保守。
        """
        forward = dist_m >= 0
        p0 = self.pose()
        if p0 is None:
            print('  ✘ 没有 /odom');  return False
        sx, sy, yaw = p0
        tx = sx + dist_m * math.cos(yaw)
        ty = sy + dist_m * math.sin(yaw)
        t0 = time.time()
        cmd = Twist()
        v = V_LINEAR if forward else -V_LINEAR * 0.6
        r = rospy.Rate(CTRL_HZ)
        last_print = 0
        while not rospy.is_shutdown():
            self.beat()
            p = self.pose()
            if p is None:
                break
            dx, dy = tx - p[0], ty - p[1]
            remain = math.hypot(dx, dy)
            if remain < 0.03:
                break
            if time.time() - t0 > timeout:
                print('  ⚠️ 前进超时（还剩 %.2f m）' % remain)
                break
            if forward:
                fm = self.front_min(0.0)
                if fm is not None and fm < FRONT_STOP:
                    print('  ★ 前方 %.2f m 有障碍 → 急停（距离还差 %.2f m）'
                          % (fm, remain))
                    break
            cmd.linear.x = math.copysign(min(abs(v), remain * 2.0 + 0.05), v)
            if not self.dry:
                self.pub.publish(cmd)
            r.sleep()
        self.stop()
        p = self.pose()
        if p:
            moved = math.hypot(p[0] - sx, p[1] - sy)
            print('  移动完成：指令 %.2f m，实际 %.2f m'
                  % (abs(dist_m), moved))
        return True


def status(d):
    for _ in range(50):
        if d.pose() and d.scan:
            break
        time.sleep(0.1)
    p = d.pose()
    if p is None:
        print('  ✘ 没有 /odom —— 底盘驱动没起？')
    else:
        print('  位姿 x=%.3f y=%.3f yaw=%.1f°'
              % (p[0], p[1], math.degrees(p[2])))
    fm = d.front_min(0.0)
    print('  前方 ±%d° 最近距离: %s'
          % (FRONT_HALF, ('%.3f m' % fm) if fm is not None else '（无有效点）'))
    if fm is not None and fm < FRONT_STOP:
        print('  ⚠️ 前方 %.2f m 已小于护栏 %.2f m —— 不能前进'
              % (fm, FRONT_STOP))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true', help='不发速度，只算')
    ap.add_argument('--status', action='store_true')
    ap.add_argument('--rotate', type=float, default=None, help='转多少度')
    ap.add_argument('--forward', type=float, default=None, help='前进多少米')
    ap.add_argument('--goto', nargs=3, type=float, default=None,
                    metavar=('X', 'Y', 'YAW_DEG'),
                    help='相对当前位姿走到 (x, y) 并朝向 yaw')
    args = ap.parse_args()

    rospy.init_node('safe_driver', anonymous=True)
    d = SafeDriver(dry_run=args.dry_run)

    try:
        if args.status:
            status(d)
        elif args.rotate is not None:
            d.rotate(args.rotate)
        elif args.forward is not None:
            d.forward(args.forward)
        elif args.goto is not None:
            x, y, yaw = args.goto
            p = d.pose()
            if p is None:
                print('  没有 /odom');  return 1
            # 先转向目标方向，再直行
            want = math.degrees(math.atan2(y, x))
            cur = math.degrees(p[2])
            d.rotate(norm_angle(math.radians(want - cur)) * 57.2957795)
            d.forward(math.hypot(x, y))
            p2 = d.pose()
            if p2 is not None:
                d.rotate(norm_angle(math.radians(yaw) - p2[2])
                         * 57.2957795)
        else:
            ap.print_help()
    finally:
        d.stop('程序退出')
    return 0


if __name__ == '__main__':
    sys.exit(main())
