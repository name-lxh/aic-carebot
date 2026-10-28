# -*- coding: utf-8 -*-
"""静止时测 /odom 的漂移 —— 决定里程计能不能当"闭环反馈"用。

⚠️ 测的时候**不要碰机器人**。如果人来人往/地面震动，读数会抖。
"""
from __future__ import print_function

import math
import time

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node('odom_drift', anonymous=True)
    try:
        m0 = rospy.wait_for_message('/odom', Odometry, timeout=10)
    except Exception as e:
        print('读不到 /odom: %r' % e)
        return 1

    def yaw_of(m):
        q = m.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    x0 = m0.pose.pose.position.x
    y0 = m0.pose.pose.position.y
    a0 = yaw_of(m0)
    print('起点: x=%.4f y=%.4f yaw=%.2f°'
          % (x0, y0, math.degrees(a0)))

    samples = []
    t0 = time.time()
    while time.time() - t0 < 10.0 and not rospy.is_shutdown():
        try:
            m = rospy.wait_for_message('/odom', Odometry, timeout=2)
            samples.append((m.pose.pose.position.x,
                            m.pose.pose.position.y, yaw_of(m)))
        except Exception:
            pass
        time.sleep(0.1)

    if not samples:
        print('★ 10 秒内一个样本都没收到')
        return 1

    dx = [s[0] - x0 for s in samples]
    dy = [s[1] - y0 for s in samples]
    da = [math.degrees(s[2] - a0) for s in samples]
    print('样本数 %d' % len(samples))
    print('位移漂移: dx %+.4f ~ %+.4f m   dy %+.4f ~ %+.4f m'
          % (min(dx), max(dx), min(dy), max(dy)))
    print('角度漂移: %+.2f° ~ %+.2f°' % (min(da), max(da)))
    span = max(math.hypot(a, b) for a, b in zip(dx, dy))
    print('最大位移 %.4f m，最大角偏 %.2f°' % (span, max(abs(v) for v in da)))
    if span < 0.02 and max(abs(v) for v in da) < 2.0:
        print('✔ 静止漂移很小，里程计可以作为闭环反馈')
    elif span < 0.10:
        print('· 有轻微漂移（%.3f m），短距离闭环可用' % span)
    else:
        print('★ 漂移偏大（%.3f m）—— 里程计闭环会不准，要考虑其他办法' % span)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
