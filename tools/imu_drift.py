# -*- coding: utf-8 -*-
"""测 IMU 的静止漂移 —— 关键看**航向(yaw)**能不能信。

轮式里程计的 yaw 实测静止漂 44°，不可用。
IMU 若带磁力计（9 轴），航向是**绝对方位**，不会随时间积分漂移。

⚠️ 磁力计在有铁/电机的机器人上容易被干扰，所以要实测。
"""
from __future__ import print_function

import math
import time

import rospy
from sensor_msgs.msg import Imu


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node('imu_drift', anonymous=True)

    topic = None
    for t in ('/imu/data', '/imu', '/imu_data', '/imu_data_corrected'):
        try:
            rospy.wait_for_message(t, Imu, timeout=4)
            topic = t
            break
        except Exception:
            continue
    if topic is None:
        print('★ 找不到可用的 IMU 话题（试过 /imu/data /imu /imu_data '
              '/imu_data_corrected）')
        return 1
    print('用话题: %s' % topic)

    m0 = rospy.wait_for_message(topic, Imu, timeout=8)
    y0 = quat_to_yaw(m0.orientation)
    print('起始 yaw = %.2f°' % math.degrees(y0))

    ys = []
    t0 = time.time()
    while time.time() - t0 < 30.0 and not rospy.is_shutdown():
        try:
            m = rospy.wait_for_message(topic, Imu, timeout=2)
            ys.append(quat_to_yaw(m.orientation))
        except Exception:
            pass
        time.sleep(0.05)

    if not ys:
        print('★ 30 秒收不到样本')
        return 1

    d = [math.degrees(y - y0) for y in ys]
    print('样本 %d 个（约 %.1f Hz）' % (len(ys), len(ys) / 30.0))
    print('yaw 漂移: %+.2f° ~ %+.2f°   总变幅 %.2f°'
          % (min(d), max(d), max(d) - min(d)))

    span = max(d) - min(d)
    if span < 2.0:
        print('✔ IMU 航向很稳（变幅 %.2f°）—— 可以用它做转向闭环' % span)
    elif span < 8.0:
        print('· IMU 航向基本可用（变幅 %.2f°）' % span)
    else:
        print('★ IMU 航向也不稳（变幅 %.2f°）—— 可能没融合磁力计或被干扰' % span)

    # 顺带看看有没有角速度
    lx = [abs(m.angular_velocity.x) for m in
          [rospy.wait_for_message(topic, Imu, timeout=3)
           for _ in range(5)]]
    print('角速度绝对值样本: %s' % ', '.join('%.4f' % v for v in lx))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
