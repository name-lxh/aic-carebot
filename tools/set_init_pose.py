# -*- coding: utf-8 -*-
"""set_init_pose.py X Y YAW -- publish /initialpose and check AMCL."""
from __future__ import print_function
import math
import sys
import time
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rospy.init_node('setinit', anonymous=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                      queue_size=1, latch=True)
time.sleep(1.0)
p = PoseWithCovarianceStamped()
p.header.frame_id = 'map'
p.header.stamp = rospy.Time.now()
p.pose.pose.position.x = x
p.pose.pose.position.y = y
q = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw))
p.pose.pose.orientation.x = q[0]
p.pose.pose.orientation.y = q[1]
p.pose.pose.orientation.z = q[2]
p.pose.pose.orientation.w = q[3]
p.pose.covariance = [0.09,0,0,0,0,0, 0,0.09,0,0,0,0, 0,0,0,0,0,0,
                     0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.03]
for _ in range(6):
    pub.publish(p)
    time.sleep(0.3)
print('initialpose published: (%.3f, %.3f) yaw=%.1f' % (x, y, yaw))
time.sleep(3.0)
try:
    ap = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped,
                                timeout=10)
    pp = ap.pose.pose.position
    print('AMCL converged to: (%.3f, %.3f)' % (pp.x, pp.y))
except Exception as e:
    print('no /amcl_pose: %r' % e)
