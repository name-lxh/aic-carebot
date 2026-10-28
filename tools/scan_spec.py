# -*- coding: utf-8 -*-
"""扫一帧 /scan，核对规格并输出用于推导场地几何的信息。

⚠️ 这个文件单独放在外面（不用 heredoc）是因为：
   heredoc 里的 Python 代码在 Py2 下**没有编码声明**，
   只要含中文注释就会：
       SyntaxError: Non-ASCII character '\\xe2' in file <stdin> on line 7,
                    but no encoding declared
   本项目已经踩过好几次。**写成文件最稳。**
"""
from __future__ import print_function

import math

import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('scan_spec', anonymous=True)
    try:
        s = rospy.wait_for_message('/scan', LaserScan, timeout=15)
    except Exception as e:
        print('读不到 /scan: %r' % e)
        return 1

    n = len(s.ranges)
    print('frame_id   = %s' % s.header.frame_id)
    print('点数       = %d   （M10 应为 1008）' % n)
    print('angle_min  = %.4f rad' % s.angle_min)
    print('angle_max  = %.4f rad' % s.angle_max)
    print('increment  = %.6f rad = %.4f deg'
          % (s.angle_increment, s.angle_increment * 57.2957795))
    print('range_min  = %.2f m' % s.range_min)
    print('range_max  = %.2f m' % s.range_max)

    fin = [v for v in s.ranges if v == v and v != float('inf') and v > 0.01]
    if not fin:
        print('★ 没有有效点 —— 雷达可能没转 / 串口没通 / 数据全 0')
        return 1
    fs = sorted(fin)
    print('有效点数    = %d / %d' % (len(fin), n))
    print('距离 min/p5/中位/p95/max = %.3f / %.3f / %.3f / %.3f / %.3f m'
          % (fs[0], fs[len(fs) // 20], fs[len(fs) // 2],
             fs[len(fs) * 19 // 20], fs[-1]))

    # 分 8 个方向给出最近的墙（角度用**激光坐标系**，0° 是正前方）
    print('')
    print('按 45° 分区，每个扇区里的最近距离：')
    nb = 8
    per = n // nb
    for b in range(nb):
        seg = s.ranges[b * per:(b + 1) * per]
        seg = [v for v in seg if v == v and v != float('inf') and v > 0.01]
        if not seg:
            print('    %3d°~%3d°   ★ 无有效点'
                  % (int(math.degrees(s.angle_min + b * per
                                      * s.angle_increment)),
                     int(math.degrees(s.angle_min + (b + 1) * per
                                      * s.angle_increment))))
            continue
        a0 = math.degrees(s.angle_min + b * per * s.angle_increment)
        a1 = math.degrees(s.angle_min + (b + 1) * per * s.angle_increment)
        print('    %6.1f°~%6.1f°   最近 %.3f m   最远 %.3f m'
              % (a0, a1, min(seg), max(seg)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
