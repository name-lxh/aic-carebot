#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""vel_watchdog.py —— /cmd_vel 看门狗：没人发非零指令时持续发零，防止底盘锁存速度后失控。

背景（2026-10-23 实测）：这台底盘固件会**锁存最后一条速度指令**，没有超时保护。
只要发速度的进程消失（任务被 kill、master 被别组清掉、手动测试脚本退出），
车就会保持最后的速度一直走——已经因此撞进过别组的测试区。

本节点：
  * 订阅 /cmd_vel，只把**非零**指令记为"活着"（自己发的零不会刷新它）；
  * 超过 0.4 s 没有非零指令 → 以 20 Hz 持续发布零速度；
  * 一旦又出现非零指令（move_base 正常下发），立刻停止补零。

用法： python2 -u vel_watchdog.py [--timeout 0.4]
"""
from __future__ import print_function

import argparse

import rospy
from geometry_msgs.msg import Twist

EPS = 1e-6


def nonzero(twist):
    return (abs(twist.linear.x) > EPS or abs(twist.linear.y) > EPS
            or abs(twist.linear.z) > EPS or abs(twist.angular.x) > EPS
            or abs(twist.angular.y) > EPS or abs(twist.angular.z) > EPS)


class Watchdog(object):
    def __init__(self, timeout=0.4):
        self.timeout = timeout
        self.last_nonzero = 0.0          # 最后一次非零指令的时间
        self.was_stale = False
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self._cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.05), self._tick)   # 20 Hz

    def _cb(self, msg):
        if nonzero(msg):
            self.last_nonzero = rospy.get_time()

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        age = rospy.get_time() - self.last_nonzero
        stale = age > self.timeout
        if stale and not self.was_stale:
            print(u'[wd] %.1f s 没有非零指令 -> 持续发零速度' % age)
        elif not stale and self.was_stale:
            print(u'[wd] 收到新的非零指令 -> 停止补零')
        self.was_stale = stale
        if stale:
            self.pub.publish(Twist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=0.4)
    args = ap.parse_args()
    rospy.init_node('vel_watchdog', anonymous=True)
    Watchdog(timeout=args.timeout)
    rospy.spin()


if __name__ == '__main__':
    main()
