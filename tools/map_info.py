# -*- coding: utf-8 -*-
"""map_info.py —— 看 /map 的当前状态（已建了多少）"""
from __future__ import print_function

import sys

import rospy
from nav_msgs.msg import OccupancyGrid


def main():
    rospy.init_node('map_info', anonymous=True)
    try:
        m = rospy.wait_for_message('/map', OccupancyGrid, timeout=15)
    except Exception as e:
        print('读不到 /map: %r' % e)
        return 1

    w, h, r = m.info.width, m.info.height, m.info.resolution
    ox = m.info.origin.position.x
    oy = m.info.origin.position.y
    d = m.data
    free = sum(1 for v in d if v == 0)
    occ = sum(1 for v in d if v > 50)
    unk = sum(1 for v in d if v < 0)

    print('地图 %d x %d 格 @ %.3f m/格 = %.1f x %.1f m'
          % (w, h, r, w * r, h * r))
    print('原点 (%.2f, %.2f)' % (ox, oy))
    print('空闲 %d   占用 %d   未知 %d   共 %d' % (free, occ, unk, len(d)))
    known = free + occ
    print('已知占比 %.1f%%' % (100.0 * known / max(1, len(d))))
    print('')
    print('地图范围：x %.2f ~ %.2f   y %.2f ~ %.2f'
          % (ox, ox + w * r, oy, oy + h * r))

    # ASCII 缩略图
    print('')
    print('ASCII 缩略图（# 占用, . 空闲, 空 未知；机器人起点在中心附近）')
    step = max(1, w // 70)
    for row in range(h - 1, -1, -step):
        line = ''
        for col in range(0, w, step):
            v = d[row * w + col]
            line += (' ' if v < 0 else ('#' if v > 50 else '.'))
        if line.strip():
            print('    |%s|' % line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
