# -*- coding: utf-8 -*-
"""
m16_tiny_move.py —— 极小步位移测试：验证"命令 -> 运动 -> odom"方向是否正确

## 背景

之前用 move_base 导航时，目标在西北、机器人却往东南跑。
已经用"全局扫描匹配"确认 **odom 与真实运动一致**，
所以问题在定位/地图。但为了排除"底盘 cmd_vel 的 x 方向反了"这种低级错误，
先用**最小动作**验证执行链路：

  1. 设初始位姿（全局匹配给出）
  2. 记录 odom 与前方净空
  3. 发 0.06 m/s 前进 2 秒（约 0.12 m）
  4. 比较 odom 位移方向 vs 车头朝向

0.12 m 的位移在任何情况下都不至于撞墙。

## 为什么写成文件而不是 heredoc

本项目反复踩这个坑：Py2 下 `python - <<EOF` 里的代码只要含中文，
就会 `SyntaxError: Non-ASCII character ... but no encoding declared`。
**写成 .py 文件并带 `# -*- coding: utf-8 -*-` 才可靠。**
"""
from __future__ import print_function

import io
import math
import sys
import time

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

INIT_X = 0.52
INIT_Y = -1.14
INIT_YAW = 195.0


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def publish_initial_pose(x, y, yaw_deg):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                          queue_size=1, latch=True)
    time.sleep(1.0)
    p = PoseWithCovarianceStamped()
    p.header.frame_id = 'map'
    p.header.stamp = rospy.Time.now()
    p.pose.pose.position.x = x
    p.pose.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(0, 0,
                                                 math.radians(yaw_deg))
    p.pose.pose.orientation.x = q[0]
    p.pose.pose.orientation.y = q[1]
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]
    cov = [0.25, 0, 0, 0, 0, 0,
           0, 0.25, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0.0685]
    p.pose.covariance = cov
    for _ in range(6):
        pub.publish(p)
        time.sleep(0.3)
    print('  已发布 /initialpose (%.2f, %.2f) yaw=%.0f'
          % (x, y, yaw_deg))


def read_odom():
    m = rospy.wait_for_message('/odom_clean', Odometry, timeout=10)
    p = m.pose.pose.position
    return p.x, p.y, quat_yaw(m.pose.pose.orientation)


def front_clear():
    s = rospy.wait_for_message('/scan_filtered', LaserScan, timeout=10)
    front = []
    allv = []
    for i, v in enumerate(s.ranges):
        if v != v or v == float('inf') or v < 0.02:
            continue
        allv.append(v)
        a = s.angle_min + i * s.angle_increment
        if abs(a) < math.radians(35.0):
            front.append(v)
    return (min(front) if front else -1.0), (min(allv) if allv else -1.0)


def main():
    rospy.init_node('tiny_move', anonymous=True)
    print('=' * 66)
    print('  极小步位移测试：命令 -> 运动 -> odom 方向一致性')
    print('=' * 66)

    print('')
    print('---- 1. 设初始位姿 ----')
    publish_initial_pose(INIT_X, INIT_Y, INIT_YAW)
    time.sleep(3.0)
    try:
        ap = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped,
                                    timeout=10)
        pp = ap.pose.pose.position
        print('  AMCL 收敛到: (%.3f, %.3f)' % (pp.x, pp.y))
    except Exception as e:
        print('  拿不到 /amcl_pose: %r' % e)

    print('')
    print('---- 2. 运动前 ----')
    x0, y0, a0 = read_odom()
    fc, ac = front_clear()
    print('  odom: x=%.4f y=%.4f yaw=%.1f°'
          % (x0, y0, math.degrees(a0)))
    print('  前方(±35°)最近 %.2f m   全向最近 %.2f m' % (fc, ac))
    if fc >= 0 and fc < 0.35:
        print('  [!] 前方太近（%.2f m），不做位移测试' % fc)
        return 1

    print('')
    print('---- 3. 前进 0.06 m/s × 2 s（约 0.12 m）----')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    r = rospy.Rate(10)
    t0 = time.time()
    while time.time() - t0 < 2.0 and not rospy.is_shutdown():
        t = Twist()
        t.linear.x = 0.06
        pub.publish(t)
        r.sleep()
    for _ in range(6):
        pub.publish(Twist())
        time.sleep(0.1)
    print('  已停止发速度')
    time.sleep(2.0)

    print('')
    print('---- 4. 运动后 ----')
    x1, y1, a1 = read_odom()
    fc2, ac2 = front_clear()
    dx, dy = x1 - x0, y1 - y0
    d = math.hypot(dx, dy)
    print('  odom: x=%.4f y=%.4f (dz yaw %.1f°)'
          % (x1, y1, math.degrees(a1 - a0)))
    print('  位移 (%+.4f, %+.4f)  长度 %.4f m' % (dx, dy, d))
    print('  前方(±35°)最近 %.2f m（原 %.2f）' % (fc2, fc))

    print('')
    print('---- 5. 结论 ----')
    if d < 0.01:
        print('  [?] 几乎没动（%.4f m）—— 底盘没响应 /cmd_vel？' % d)
        return 1
    moved = math.degrees(math.atan2(dy, dx))
    head = math.degrees(a0)
    diff = (moved - head + 180) % 360 - 180
    print('  位移方向 %.1f°   车头朝向 %.1f°   夹角 %+.1f°'
          % (moved, head, diff))
    if abs(diff) < 40:
        print('  [OK] 朝车头方向前进 —— 执行链路方向正确')
        print('       (导航跑偏的原因在定位/地图，不在底盘)')
        return 0
    elif abs(abs(diff) - 180) < 50:
        print('  [!!] 朝反方向走 —— 底盘 cmd_vel 的 x 符号反了！')
        return 2
    else:
        print('  [??] 方向偏差 %.0f°，需要继续查' % diff)
        return 3


if __name__ == '__main__':
    sys.exit(main())
