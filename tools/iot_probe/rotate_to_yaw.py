#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Rotate the base to a target yaw (rad) using /odom feedback, then stop."""
import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Rotator(object):
    def __init__(self, target):
        self.target = target
        self.yaw = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = 2.0 * math.atan2(q.z, q.w)

    def run(self):
        rate = rospy.Rate(10)
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.yaw is None:
                rate.sleep()
                continue
            err = self.target - self.yaw
            while err > math.pi:
                err -= 2 * math.pi
            while err < -math.pi:
                err += 2 * math.pi
            rospy.loginfo('yaw=%.1f deg target=%.1f deg err=%.1f deg',
                          math.degrees(self.yaw), math.degrees(self.target),
                          math.degrees(err))
            if abs(err) < math.radians(2.0):
                break
            cmd = Twist()
            cmd.angular.z = max(-0.18, min(0.18, 1.2 * err))
            self.pub.publish(cmd)
            rate.sleep()
        for _ in range(5):
            self.pub.publish(Twist())
            rospy.sleep(0.05)


if __name__ == '__main__':
    rospy.init_node('rotate_to_yaw')
    target_deg = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    Rotator(math.radians(target_deg)).run()
    rospy.loginfo('done')
