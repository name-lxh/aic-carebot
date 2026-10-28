# -*- coding: utf-8 -*-
"""grab_pose.py -- read map->base_footprint (gmapping truth) and save it."""
from __future__ import print_function
import math
import rospy
import tf

rospy.init_node('grabpose', anonymous=True)
lst = tf.TransformListener()
rospy.sleep(1.0)
got = None
for _ in range(40):
    try:
        lst.waitForTransform('/map', '/base_footprint', rospy.Time(0),
                             rospy.Duration(1.0))
        (tr, rot) = lst.lookupTransform('/map', '/base_footprint',
                                        rospy.Time(0))
        yaw = math.atan2(2 * (rot[3] * rot[2] + rot[0] * rot[1]),
                         1 - 2 * (rot[1] * rot[1] + rot[2] * rot[2]))
        got = (tr[0], tr[1], math.degrees(yaw))
        break
    except Exception:
        rospy.sleep(0.5)
if got is None:
    print('SELECT_FAIL')
else:
    print('gmapping pose: x=%.4f y=%.4f yaw=%.2f' % got)
    with open('/tmp/grabbed_pose.txt', 'w') as f:
        f.write('%.6f %.6f %.6f\n' % got)
    print('saved /tmp/grabbed_pose.txt')
