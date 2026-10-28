# -*- coding: utf-8 -*-
"""m1_scan_check.py —— 测 /scan 规格 + 近距噪声（为建图和安全护栏定参）"""
from __future__ import print_function

import math

import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('m1_scan', anonymous=True)
    try:
        s = rospy.wait_for_message('/scan', LaserScan, timeout=12)
    except Exception as e:
        print('读不到 /scan: %r' % e)
        return 1

    n = len(s.ranges)
    print('frame=%s  点数=%d' % (s.header.frame_id, n))
    print('angle_min=%.4f  angle_max=%.4f  增量=%.5f rad (%.3f°)'
          % (s.angle_min, s.angle_max, s.angle_increment,
             math.degrees(s.angle_increment)))
    print('range_min=%.2f  range_max=%.2f' % (s.range_min, s.range_max))

    inf = sum(1 for v in s.ranges if v == float('inf'))
    nan = sum(1 for v in s.ranges if v != v)
    zero = sum(1 for v in s.ranges if v == 0.0)
    fin = [v for v in s.ranges if v == v and v != float('inf') and v > 0.02]
    print('inf=%d  nan=%d  0.0=%d  有效=%d' % (inf, nan, zero, len(fin)))
    if not fin:
        print('[FAIL] 没有有效点')
        return 1

    fs = sorted(fin)
    print('距离 min/p5/中位/p95/max = %.3f / %.3f / %.3f / %.3f / %.3f m'
          % (fs[0], fs[len(fs)//20], fs[len(fs)//2],
             fs[len(fs)*19//20], fs[-1]))

    near = [v for v in fin if v < 0.30]
    print('近距点数(<0.30m) = %d  (%.1f%%)'
          % (len(near), 100.0 * len(near) / len(fin)))
    if len(near) > len(fin) * 0.10:
        print('[!] 近距点偏多 —— 像自扫/地噪，护栏要按"簇"判断而不是单点')

    # 每 45° 最近距离（判断周围空间）
    print('')
    print('每 45° 扇区最近距离:')
    seg = max(1, n // 8)
    for b in range(8):
        part = [v for v in s.ranges[b*seg:(b+1)*seg]
                if v == v and v != float('inf') and v > 0.02]
        a0 = math.degrees(s.angle_min + b*seg*s.angle_increment)
        a1 = math.degrees(s.angle_min + min((b+1)*seg, n-1)*s.angle_increment)
        if part:
            print('    %6.1f~%6.1f°  最近 %6.3f  中位 %6.3f  点数 %3d'
                  % (a0, a1, min(part), sorted(part)[len(part)//2], len(part)))
        else:
            print('    %6.1f~%6.1f°  无有效点' % (a0, a1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
