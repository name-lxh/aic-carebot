# -*- coding: utf-8 -*-
"""diag_base2.py —— 用**激光**独立验证底盘是否真的移动（不依赖开环 odom）

    python2 diag_base2.py [速度] [时长]

## 为什么不能只看 odom

这台车的 `/odom_clean` 由 `odom_imu.py` 生成：平移是**按 /cmd_vel 命令积分**的（开环）。
所以"odom 走了 0.1 m"只能说明指令发出去了，**不能**证明轮子转了。

## 这个脚本怎么判

1. 抓一帧 `/scan`，记下 正前方 ±20° 的最近距离
2. 直接发 `/cmd_vel` 前进
3. 再抓一帧，比较正前方距离变化 Δrange
4. 判据：
     车真的前进了 D 米  ->  正前方距离应减少约 D（Δrange ≈ -D）
     车没动            ->  Δrange ≈ 0

在开阔处（前方 >2 m 无障碍）这个判据不灵敏，所以脚本会先看前方是否 0.5~2.5 m 有墙，
不是就提示换个朝向或用 --force。
"""
from __future__ import print_function

import math
import sys

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SELF_BANDS = ((15, 35), (145, 215))


def front_min(scan, cone_deg=20.0):
    best = 99.0
    for i, v in enumerate(scan.ranges):
        if not (v == v) or v < 0.36 or v > 6.0:
            continue
        a = math.degrees(math.atan2(math.sin(scan.angle_min + i * scan.angle_increment),
                                    math.cos(scan.angle_min + i * scan.angle_increment)))
        d = abs(((a + 180.0) % 360.0) - 180.0)
        if any(lo <= d <= hi for lo, hi in SELF_BANDS) and v < 0.50:
            continue
        if abs(a) <= cone_deg and v < best:
            best = v
    return best


def profile(scan, nbins=24):
    """把一圈按 15° 分箱取最近距离，用来做整体相似度对比。"""
    bins = [99.0] * nbins
    for i, v in enumerate(scan.ranges):
        if not (v == v) or v < 0.36 or v > 8.0:
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        k = int(((a + 360.0) % 360.0) // (360.0 / nbins)) % nbins
        if v < bins[k]:
            bins[k] = v
    return bins


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
    force = '--force' in sys.argv

    rospy.init_node('diag_base2', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    s0 = rospy.wait_for_message('/scan', LaserScan, timeout=10)
    f0 = front_min(s0)
    p0 = profile(s0)
    print('  正前方最近 %.2f m' % f0)
    if not force and not (0.5 <= f0 <= 2.5):
        print('  [注意] 前方 %.2f m，不在 0.5~2.5 m，位移判据不灵敏；' % f0)
        print('         把车对着 1 m 左右的墙再跑，或加 --force 强行测（只看 odom）')

    D = speed * dur
    print('')
    print('  发 /cmd_vel %.2f m/s × %.1f s（目标位移 %.2f m）' % (speed, dur, D))
    tw = Twist()
    tw.linear.x = speed
    t0 = rospy.Time.now()
    rate = rospy.Rate(20)
    while (rospy.Time.now() - t0).to_sec() < dur and not rospy.is_shutdown():
        pub.publish(tw)
        rate.sleep()
    tw.linear.x = 0.0
    for _ in range(8):
        pub.publish(tw)
        rospy.sleep(0.05)
    rospy.sleep(0.5)

    s1 = rospy.wait_for_message('/scan', LaserScan, timeout=10)
    f1 = front_min(s1)
    p1 = profile(s1)
    drange = f1 - f0

    diff = sum(abs(a - b) for a, b in zip(p0, p1) if a < 90 and b < 90)
    same = sum(1 for a, b in zip(p0, p1) if a < 90 and b < 90)

    print('')
    print('  正前方 %.2f m -> %.2f m   Δrange = %+.3f m' % (f0, f1, drange))
    print('  分箱剖面平均变化 %.3f m（%d 个有效箱）'
          % (diff / max(same, 1), same))
    print('')
    if same == 0:
        print('  [FAIL] 激光几乎看不到有效面，无法判断')
        return 1
    if drange < -0.4 * D:
        print('  ✅ 车**真的前进了**（正前方距离缩短 %+.2f m，约等于目标 %.2f m）'
              % (drange, D))
        return 0
    if abs(drange) < 0.03:
        print('  ❌ 车**没有动**：激光看到的前方距离几乎没变（%+.3f m）' % drange)
        print('     说明 /cmd_vel 没有被底盘执行 —— 查电池电压 / 急停 / stm32 连接')
        return 2
    print('  ⚠ 结果不明确（Δrange=%+.2f m，目标 %.2f m），换个朝向再测' % (drange, D))
    return 3


if __name__ == '__main__':
    sys.exit(main())
