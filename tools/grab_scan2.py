# -*- coding: utf-8 -*-
"""grab_scan2.py —— 同时抓 /scan(原始) 与 /scan_filtered，存 npz（只读）。"""
from __future__ import print_function

import sys

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


def dump(topic, timeout=15):
    s = rospy.wait_for_message(topic, LaserScan, timeout=timeout)
    r = np.array(s.ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r > 0.02)
    print('%-16s %4d 点  有效 %3d  最近 %.3f  最远 %.3f  frame=%s'
          % (topic, len(r), ok.sum(), r[ok].min() if ok.any() else -1,
             r[ok].max() if ok.any() else -1, s.header.frame_id))
    return dict(ranges=r, angle_min=s.angle_min, angle_increment=s.angle_increment,
                range_min=s.range_min, range_max=s.range_max)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/scan2.npz'
    rospy.init_node('grab_scan2', anonymous=True)
    d = {}
    for t, k in (('/scan', 'raw'), ('/scan_filtered', 'flt')):
        try:
            s = dump(t)
            for kk, v in s.items():
                d['%s_%s' % (k, kk)] = v
        except Exception as e:
            print('[warn] %s: %r' % (t, e))
    np.savez(out, **d)
    print('saved -> %s' % out)


if __name__ == '__main__':
    main()
