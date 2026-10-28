# -*- coding: utf-8 -*-
"""diag_base.py —— 底盘"能不能动"的判定脚本（充电后先跑这个）

## 为什么需要

今天反复出现"发了目标车不动"，可能的原因有三类，必须分开：

    A. 规划层没发指令     —— /cmd_vel 上没有数据
    B. 发了但底盘不执行   —— /cmd_vel 有数据，但 /feedback_vel 仍为 0
                              （电池电压低、急停按下、驱动掉线、电机保护）
    C. 底盘在执行但"没动" —— /feedback_vel 有速度，实际位移为 0

本脚本直接测 B/C：绕开 move_base，**直接发 /cmd_vel**，同时对比
`/odom_clean` 的位移与 `/feedback_vel` 的轮速。

## 用法（车上）

    python2 _tools/real_test/diag_base.py            # 前进 0.05 m/s × 2 s
    python2 _tools/real_test/diag_base.py 0.08 3     # 自定义
"""
from __future__ import print_function

import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0


def main():
    rospy.init_node('diag_base', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    print('=== 1) 电池/底盘状态话题 ===')
    topics = [t for t in rospy.get_published_topics()]
    hits = [t for t, _ in topics if any(k in t.lower() for k in
                                        ('batt', 'volt', 'power', 'chassis', 'base', 'stm32', 'feedback'))]
    for t in sorted(hits):
        try:
            msg = rospy.wait_for_message(t, rospy.AnyMsg, timeout=2.0)
            print('  %-32s %s' % (t, str(msg)[:90].replace('\n', ' ')))
        except Exception:
            print('  %-32s (无数据)' % t)
    if not hits:
        print('  （没有电池/底盘状态话题）')

    print('')
    print('=== 2) 基线：odom / feedback_vel ===')
    o0 = None
    try:
        o0 = rospy.wait_for_message('/odom_clean', Odometry, timeout=5)
        print('  /odom_clean 位置 (%.4f, %.4f)' % (o0.pose.pose.position.x, o0.pose.pose.position.y))
    except Exception as exc:
        print('  [FAIL] 拿不到 /odom_clean: %r' % exc)
        return 1

    fb = None
    for t in ('/feedback_vel', '/odom', '/wheel_vel'):
        try:
            fb = rospy.wait_for_message(t, rospy.AnyMsg, timeout=1.5)
            print('  %s 有数据: %s' % (t, str(fb)[:80].replace('\n', ' ')))
            break
        except Exception:
            continue
    if fb is None:
        print('  （没找到轮速反馈话题）')

    print('')
    print('=== 3) 直接发 /cmd_vel %.2f m/s × %.1f s（不经过 move_base）===' % (SPEED, DUR))
    tw = Twist()
    tw.linear.x = SPEED
    t0 = rospy.Time.now()
    rate = rospy.Rate(20)
    while (rospy.Time.now() - t0).to_sec() < DUR and not rospy.is_shutdown():
        pub.publish(tw)
        rate.sleep()
    # 停车 + 连续发 0.3 s，避免底盘保持上一条速度
    tw.linear.x = 0.0
    for _ in range(6):
        pub.publish(tw)
        rospy.sleep(0.05)

    print('')
    print('=== 4) 结果 ===')
    try:
        o1 = rospy.wait_for_message('/odom_clean', Odometry, timeout=5)
    except Exception as exc:
        print('  [FAIL] 拿不到结束位姿: %r' % exc)
        return 1
    dx = o1.pose.pose.position.x - o0.pose.pose.position.x
    dy = o1.pose.pose.position.y - o0.pose.pose.position.y
    dist = math.hypot(dx, dy)
    print('  位移 (%.4f, %.4f)  长度 %.4f m' % (dx, dy, dist))
    print('  odom 当前速度: %.3f m/s' % o1.twist.twist.linear.x)

    expect = SPEED * DUR
    print('')
    if dist > max(0.02, expect * 0.3):
        print('  ✅ 底盘会动（期望 %.3f m，实际 %.3f m）—— 之前"不动"是规划层的事' % (expect, dist))
        return 0
    print('  ❌ 底盘没动（期望 %.3f m，实际 %.3f m）—— 问题在底盘/供电，不在导航' % (expect, dist))
    print('     排查顺序：')
    print('       1) 电池电压（万用表或底盘状态话题）是否低于底盘保护阈值')
    print('       2) 急停/使能开关是否松开')
    print('       3) /dev/ttyUSB* 上 stm32 串口是否还在、stm32_node 进程是否活着')
    print('       4) 看 /feedback_vel 是否有非零轮速（有轮速但 odom 不动 = 编码器/odom 的事）')
    return 2


if __name__ == '__main__':
    sys.exit(main())
