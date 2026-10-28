# -*- coding: utf-8 -*-
"""scan_check.py —— 确认 gmapping 收到的 /scan 是否完整

⚠️ 疑点：地图上只有一条窄线（空闲 2565 格），但 360° 雷达
   在 3m 场地里应该看到一整圈墙。要么：
     · 扫描数据不完整（角度范围被截断）
     · 或者地图参数太大被稀释了
   本脚本直接看数据本身。
"""
from __future__ import print_function

import math
import sys

import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('scan_check', anonymous=True)
    try:
        s = rospy.wait_for_message('/scan', LaserScan, timeout=15)
    except Exception as e:
        print('读不到 /scan: %r' % e)
        return 1

    n = len(s.ranges)
    inc = s.angle_increment
    print('frame_id = %s' % s.header.frame_id)
    print('点数 %d   angle_min %.4f  angle_max %.4f  增量 %.5f'
          % (n, s.angle_min, s.angle_max, inc))
    print('跨度 %.1f°' % math.degrees(s.angle_max - s.angle_min
                                      + inc))
    print('range_min %.2f  range_max %.2f' % (s.range_min, s.range_max))

    # 有效/无效统计
    inf = sum(1 for v in s.ranges if v == float('inf'))
    nan = sum(1 for v in s.ranges if v != v)
    zero = sum(1 for v in s.ranges if v == 0.0)
    fin = [v for v in s.ranges if v == v and v != float('inf')
           and v > 0.02]
    print('')
    print('inf %d   nan %d   0.0 %d   有效 %d' % (inf, nan, zero, len(fin)))

    # 按 30° 分区统计有效点数和距离
    print('')
    print('每 30° 分区（有效点数 / 最近 / 最远）：')
    seg = max(1, n // 12)
    for b in range(12):
        part = []
        for i in range(b * seg, min((b + 1) * seg, n)):
            v = s.ranges[i]
            if v == v and v != float('inf') and v > 0.02:
                part.append(v)
        a0 = math.degrees(s.angle_min + b * seg * inc)
        a1 = math.degrees(s.angle_min
                          + min((b + 1) * seg, n - 1) * inc)
        if part:
            print('  %7.1f~%7.1f°   %4d 点   最近 %6.3f  最远 %6.3f'
                  % (a0, a1, len(part), min(part), max(part)))
        else:
            print('  %7.1f~%7.1f°   ★ 0 点' % (a0, a1))

    # TF 链是否能连通 laser_link
    import tf
    try:
        lst = tf.TransformListener()
        rospy.sleep(2.0)
        lst.waitForTransform('/map', s.header.frame_id,
                             rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = lst.lookupTransform('/map', s.header.frame_id,
                                           rospy.Time(0))
        print('')
        print('map -> %s 的变换：' % s.header.frame_id)
        print('  平移 (%.3f, %.3f, %.3f)' % trans)
    except Exception as e:
        print('')
        print('★ map -> %s 变换拿不到: %s' % (s.header.frame_id, e))
    return 0


if __name__ == '__main__':
    sys.exit(main())
