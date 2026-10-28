# -*- coding: utf-8 -*-
"""diag_base3.py —— 判断"车动不了"到底卡在哪一层

    python2 diag_base3.py

三层测试（全部用激光/轮速**独立**验证，不信开环 odom）：
  A. 轮速回声   —— 发 cmd_vel，看 /feedback_vel 是否跟得上（驱动是否在执行）
  B. 直线位移   —— 发前进，看激光前方距离是否按比例缩短（真的有位移吗）
  C. 原地旋转   —— 发角速度，看激光剖面是否整体旋转（轮子是否抓地）

结论：
  轮速 0          -> 底盘驱动没收到指令（stm32 掉线/串口）
  轮速有、位移无  -> 轮子空转/打滑/被卡住（机械或地面）
  轮速有、位移有  -> 底盘正常，问题在上层（定位/规划）
"""
from __future__ import print_function

import codecs
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

pub = None
fb_cur = [0.0]
fb_max = [0.0]


def on_fb(msg):
    fb_cur[0] = msg.linear.x
    if abs(msg.linear.x) > abs(fb_max[0]):
        fb_max[0] = msg.linear.x


def drive(vx=0.0, wz=0.0, dur=3.0):
    """发速度，返回这段时间的最大轮速。"""
    fb_max[0] = 0.0
    tw = Twist()
    tw.linear.x = vx
    tw.angular.z = wz
    t0 = time.time()
    rate = rospy.Rate(20)
    while time.time() - t0 < dur and not rospy.is_shutdown():
        pub.publish(tw)
        rate.sleep()
    tw.linear.x = 0.0
    tw.angular.z = 0.0
    for _ in range(8):
        pub.publish(tw)
        rospy.sleep(0.05)
    rospy.sleep(0.4)
    return fb_max[0]


def front_min(scan, cone=20.0):
    best = 99.0
    for i, v in enumerate(scan.ranges):
        if not (v == v) or v < 0.30 or v > 8.0:
            continue
        a = math.degrees(math.atan2(math.sin(scan.angle_min + i * scan.angle_increment),
                                    math.cos(scan.angle_min + i * scan.angle_increment)))
        d = abs(((a + 180.0) % 360.0) - 180.0)
        if 15 <= d <= 35 or 145 <= d <= 215:
            if v < 0.50:
                continue
        if abs(a) <= cone and v < best:
            best = v
    return best


def profile(scan, nbins=36):
    bins = [99.0] * nbins
    for i, v in enumerate(scan.ranges):
        if not (v == v) or v < 0.30 or v > 8.0:
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        k = int(((a + 360.0) % 360.0) // (360.0 / nbins)) % nbins
        if v < bins[k]:
            bins[k] = v
    return bins


def profile_shift(p0, p1):
    """找最佳旋转位移（箱数），用来判断是否真的转起来了。"""
    n = len(p0)
    best_k, best_err = 0, 1e9
    for k in range(n):
        err = 0.0
        cnt = 0
        for i in range(n):
            a, b = p0[i], p1[(i + k) % n]
            if a < 90 and b < 90:
                err += abs(a - b)
                cnt += 1
        if cnt >= 6:
            err /= cnt
            if err < best_err:
                best_err, best_k = err, k
    return best_k, best_err


def main():
    global pub
    rospy.init_node('diag_base3', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    rospy.Subscriber('/feedback_vel', Twist, on_fb, queue_size=5)
    rospy.sleep(1.5)

    print(u'')
    print(u'=== A. 轮速回声（驱动是否执行）===')
    v = drive(0.06, 0.0, 3.0)
    print(u'  发 0.06 m/s，轮速峰值 %.3f m/s  -> %s'
          % (v, u'驱动在执行' if v > 0.03 else u'❌ 驱动没执行（stm32/串口）'))

    print(u'')
    print(u'=== B. 直线位移（激光验证）===')
    s0 = rospy.wait_for_message('/scan', LaserScan, timeout=10)
    f0 = front_min(s0)
    print(u'  正前方最近 %.2f m' % f0)
    v2 = drive(0.06, 0.0, 4.0)
    s1 = rospy.wait_for_message('/scan', LaserScan, timeout=10)
    f1 = front_min(s1)
    dr = f1 - f0
    print(u'  前进后 %.2f m  Δ=%+.3f m（期望约 -0.24 m），轮速峰值 %.3f' % (f1, dr, v2))

    print(u'')
    print(u'=== C. 原地旋转（轮子是否抓地）===')
    p0 = profile(s0)
    v3 = drive(0.0, 0.5, 3.0)
    s2 = rospy.wait_for_message('/scan', LaserScan, timeout=10)
    p2 = profile(s2)
    k, err = profile_shift(p0, p2)
    print(u'  转 0.5 rad/s × 3 s（约 86°），剖面最佳位移 %.0f° / 误差 %.2f m，轮速峰值 %.3f'
          % (k * 10.0, err, v3))
    print(u'')
    print(u'=== 结论 ===')
    if v < 0.03:
        print(u'  ❌ 驱动层：/cmd_vel 发出去了但底盘不执行 —— 查 stm32_node / 串口 / 急停')
    elif abs(dr) < 0.06 and k <= 1:
        print(u'  ❌ 机械层：轮速有（%.3f m/s）但车既不平移也不旋转 —— 轮子空转/被打滑/被卡住' % v)
    elif abs(dr) < 0.06 and k >= 2:
        print(u'  ⚠ 只能转不能直行：轮子抓地但直行失效（单侧驱动/电机）')
    else:
        print(u'  ✅ 底盘正常（能平移 %+.2f m、能旋转 %.0f°）—— 问题在上层（定位/规划）'
              % (dr, k * 10.0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
