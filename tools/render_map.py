# -*- coding: utf-8 -*-
"""
render_map.py —— 把 /map 渲染成较宽的 ASCII 图

⚠️ 上一版缩略图每 3 格抽 1（step=w//70），256 格的地图被抽成 85 列，
   而且用 '#'/'.' 两档，看不出结构。改成：
     · 每格 1 字符，一行最多 110 字符
     · 三档密度：# 占用 / . 空闲 / 空 未知
     · 标出机器人当前位置（从 TF 拿 map->base_footprint）
"""
from __future__ import print_function

import math
import sys

import rospy
import tf
from nav_msgs.msg import OccupancyGrid


def main():
    rospy.init_node('render_map', anonymous=True)
    try:
        m = rospy.wait_for_message('/map', OccupancyGrid, timeout=15)
    except Exception as e:
        print('读不到 /map: %r' % e)
        return 1

    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    d = m.data

    free = sum(1 for v in d if v == 0)
    occ = sum(1 for v in d if v > 50)
    print('地图 %dx%d @%.3f m  = %.1f x %.1f m' % (w, h, res, w * res, h * res))
    print('原点 (%.2f, %.2f)' % (ox, oy))
    print('空闲 %d   占用 %d   未知 %d   已知 %.1f%%'
          % (free, occ, len(d) - free - occ,
             100.0 * (free + occ) / len(d)))

    # 机器人位置（TF）
    rx = ry = None
    try:
        lst = tf.TransformListener()
        rospy.sleep(2.0)
        lst.waitForTransform('/map', '/base_footprint',
                             rospy.Time(0), rospy.Duration(5.0))
        (tr, _r) = lst.lookupTransform('/map', '/base_footprint',
                                       rospy.Time(0))
        rx, ry = tr[0], tr[1]
        print('机器人在地图坐标: (%.2f, %.2f)' % (rx, ry))
    except Exception as e:
        print('拿不到机器人位置: %s' % e)

    # 只画"有内容"的范围，避免大量空白
    cols_used = [c for c in range(w) if any(
        d[r * w + c] != -1 for r in range(0, h, 2))]
    rows_used = [r for r in range(h) if any(
        d[r * w + c] != -1 for c in range(0, w, 2))]
    if not cols_used or not rows_used:
        print('地图全空（gmapping 还没积累）')
        return 0
    c0, c1 = max(0, min(cols_used) - 3), min(w - 1, max(cols_used) + 3)
    r0, r1 = max(0, min(rows_used) - 3), min(h - 1, max(rows_used) + 3)
    print('内容范围: 列 %d~%d  行 %d~%d' % (c0, c1, r0, r1))

    # 机器人所在格
    rcol = rrow = None
    if rx is not None:
        rcol = int((rx - ox) / res)
        rrow = int((ry - oy) / res)

    print('')
    print('ASCII 地图（# 占用, . 空闲, 空 未知, R 机器人；上=+y）')
    for row in range(r1, r0 - 1, -1):
        line = []
        for col in range(c0, c1 + 1):
            if rcol is not None and col == rcol and row == rrow:
                line.append('R')
                continue
            v = d[row * w + col]
            line.append(' ' if v < 0 else ('#' if v > 50 else '.'))
        s = ''.join(line)
        # 每行左侧标出该行对应的 y
        y = oy + row * res
        print('  %6.2f |%s|' % (y, s))
    # 底轴
    print('         +' + '-' * (c1 - c0 + 1) + '+')
    lab = ' ' * (c1 - c0 + 1)
    for col in range(c0, c1 + 1, 10):
        x = ox + col * res
        pos = col - c0
        t = '%.1f' % x
        lab = lab[:pos] + t + lab[pos + len(t):]
    print('          ' + lab)
    print('          (x 轴，单位 m)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
