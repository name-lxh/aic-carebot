# -*- coding: utf-8 -*-
"""m1_imu_check.py —— 测 /imu_data 航向稳定性"""
from __future__ import print_function

import math
import time

import rospy
from sensor_msgs.msg import Imu


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node('m1_imu', anonymous=True)
    t = None
    for cand in ('/imu_data', '/imu/data', '/imu'):
        try:
            rospy.wait_for_message(cand, Imu, timeout=4)
            t = cand
            break
        except Exception:
            continue
    if t is None:
        print('找不到 IMU 话题（试过 /imu_data /imu/data /imu）')
        return 1
    print('话题: %s' % t)

    m0 = rospy.wait_for_message(t, Imu, timeout=8)
    a0 = yaw(m0.orientation)
    print('起始 yaw = %.2f°   frame=%s'
          % (math.degrees(a0), m0.header.frame_id))

    vals = []
    gz = []
    t0 = time.time()
    while time.time() - t0 < 12.0 and not rospy.is_shutdown():
        try:
            m = rospy.wait_for_message(t, Imu, timeout=2)
        except Exception:
            continue
        vals.append(yaw(m.orientation))
        gz.append(m.angular_velocity.z)
    if not vals:
        print('12 秒无样本')
        return 1

    d = [math.degrees(v - a0) for v in vals]
    span = max(d) - min(d)
    print('样本 %d（约 %.1f Hz）' % (len(vals), len(vals) / 12.0))
    print('航向漂移 %+.3f° ~ %+.3f°   变幅 %.3f°'
          % (min(d), max(d), span))
    print('角速度 z: %.5f ~ %.5f rad/s'
          % (min(gz), max(gz)))
    print('')
    if span < 1.0:
        print('[OK] 航向非常稳，可以用它做转向闭环和 odom 航向源')
    elif span < 5.0:
        print('[~] 基本可用')
    else:
        print('[FAIL] 航向不稳')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
