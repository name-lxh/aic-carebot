#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Print min lidar range in rear/front sectors."""
import math

import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('scan_rear_check')
    msg = rospy.wait_for_message('/scan', LaserScan, timeout=5.0)
    n = len(msg.ranges)

    def sector_min(lo_deg, hi_deg):
        vals = []
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max):
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            ang = (ang + 180.0) % 360.0 - 180.0
            if lo_deg <= ang <= hi_deg:
                vals.append(r)
        return min(vals) if vals else None

    print('front[-30,30]=%s' % sector_min(-30, 30))
    print('rear[150,210]=%s' % sector_min(150, 210))
    print('left[60,120]=%s' % sector_min(60, 120))
    print('right[-120,-60]=%s' % sector_min(-120, -60))


if __name__ == '__main__':
    main()
