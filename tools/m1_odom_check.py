# -*- coding: utf-8 -*-
"""m1_odom_check.py —— 测 /odom 静止漂移（判断能不能直接给 gmapping 用）"""
from __future__ import print_function

import math
import time

import rospy
from nav_msgs.msg import Odometry


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node('m1_odom', anonymous=True)
    try:
        m0 = rospy.wait_for_message('/odom', Odometry, timeout=10)
    except Exception as e:
        print('读不到 /odom: %r' % e)
        return 1

    x0 = m0.pose.pose.position.x
    y0 = m0.pose.pose.position.y
    a0 = yaw(m0.pose.pose.orientation)
    print('起点  x=%.3f  y=%.3f  yaw=%.2f°'
          % (x0, y0, math.degrees(a0)))
    print('frame: %s -> %s'
          % (m0.header.frame_id, m0.child_frame_id))

    xs, ys, as_ = [], [], []
    t0 = time.time()
    while time.time() - t0 < 12.0 and not rospy.is_shutdown():
        try:
            m = rospy.wait_for_message('/odom', Odometry, timeout=2)
        except Exception:
            continue
        xs.append(m.pose.pose.position.x)
        ys.append(m.pose.pose.position.y)
        as_.append(yaw(m.pose.pose.orientation))
    if not xs:
        print('12 秒无样本')
        return 1

    dx = [v - x0 for v in xs]
    dy = [v - y0 for v in ys]
    da = [math.degrees(v - a0) for v in as_]
    span = max(math.hypot(a, b) for a, b in zip(dx, dy))
    print('样本 %d   线速度读数: %s' % (
        len(xs),
        ('%.4f' % m0.twist.twist.linear.x)))
    print('位移漂移  dx %+.3f~%+.3f  dy %+.3f~%+.3f m'
          % (min(dx), max(dx), min(dy), max(dy)))
    print('航向漂移  %+.2f° ~ %+.2f°  (变幅 %.2f°)'
          % (min(da), max(da), max(da) - min(da)))
    print('最大位移  %.3f m' % span)
    print('')
    if abs(x0) > 5 or abs(y0) > 5:
        print('[!] 位置绝对值离谱（|x|=%.1f |y|=%.1f）—— 像绝对编码器值，'
              '不是增量积分' % (abs(x0), abs(y0)))
    if span < 0.03 and (max(da) - min(da)) < 3.0:
        print('[OK] 静止时很稳，可以直接给 gmapping 用')
    elif span < 0.15 and (max(da) - min(da)) < 10.0:
        print('[~] 有轻微漂移，短时建图可接受')
    else:
        print('[FAIL] 漂移大 —— **不能**直接给 gmapping，需要另做 odom 源')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
