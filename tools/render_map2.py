# -*- coding: utf-8 -*-
"""
render_map2.py —— 聚焦渲染（只画机器人周围 / 只画有内容的区域）

⚠️ 上一版按"内容范围"取列（0~217 列 = 6.5 m），一行 200 多字符，
   在终端里折行，完全看不出结构。改成：
     · 只画机器人周围 ±R 米
     · 每 N 格抽 1，保证一行 <= 100 字符
     · 左侧标 y，底部标 x
"""
from __future__ import print_function

import math
import sys

import rospy
import tf
from nav_msgs.msg import OccupancyGrid


def main():
    R = 2.2           # 画机器人周围 ±R 米
    MAXCOLS = 100
    for i, a in enumerate(sys.argv):
        if a == '--r' and i + 1 < len(sys.argv):
            R = float(sys.argv[i + 1])

    rospy.init_node('render_map2', anonymous=True)
    try:
        m = rospy.wait_for_message('/map', OccupancyGrid, timeout=15)
    except Exception as e:
        print('读不到 /map: %r' % e)
        return 1

    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    d = m.data

    # 机器人在哪
    rx = ry = 0.0
    try:
        lst = tf.TransformListener()
        rospy.sleep(2.0)
        lst.waitForTransform('/map', '/base_footprint',
                             rospy.Time(0), rospy.Duration(5.0))
        (tr, _r) = lst.lookupTransform('/map', '/base_footprint',
                                       rospy.Time(0))
        rx, ry = tr[0], tr[1]
    except Exception as e:
        print('（拿不到机器人位置，改用地图中心）: %s' % e)

    print('地图 %dx%d @%.3f m   机器人 (%.2f, %.2f)'
          % (w, h, res, rx, ry))
    free = sum(1 for v in d if v == 0)
    occ = sum(1 for v in d if v > 50)
    print('空闲 %d  占用 %d  已知 %.1f%%'
          % (free, occ, 100.0 * (free + occ) / len(d)))

    # 取机器人周围的列/行
    c_r = int((rx - ox) / res)
    r_r = int((ry - oy) / res)
    half = int(R / res)
    c0, c1 = max(0, c_r - half), min(w - 1, c_r + half)
    r0, r1 = max(0, r_r - half), min(h - 1, r_r + half)
    ncol = c1 - c0 + 1
    step = max(1, int(math.ceil(ncol / float(MAXCOLS))))

    print('')
    print('聚焦视图 ±%.1f m（每 %d 格抽 1；# 占用  . 空闲  空 未知  R 机器人）'
          % (R, step))
    print('')
    for row in range(r1, r0 - 1, -step):
        line = []
        for col in range(c0, c1 + 1, step):
            # 该块里只要有占用就画 #
            blk_occ = False
            blk_free = False
            blk_rob = False
            for rr in range(row, max(row - step, r0 - 1), -1):
                for cc in range(col, min(col + step, c1 + 1)):
                    if cc == c_r and rr == r_r:
                        blk_rob = True
                        continue
                    v = d[rr * w + cc]
                    if v > 50:
                        blk_occ = True
                    elif v == 0:
                        blk_free = True
            if blk_rob:
                line.append('R')
            elif blk_occ:
                line.append('#')
            elif blk_free:
                line.append('.')
            else:
                line.append(' ')
        yv = oy + row * res
        print('  %6.2f |%s|' % (yv, ''.join(line)))

    # x 轴
    width = len(range(c0, c1 + 1, step))
    print('         +' + '-' * width + '+')
    lab = [' '] * width
    k = 0
    for col in range(c0, c1 + 1, step):
        if (col - c0) % (step * 10) == 0:
            t = '%.1f' % (ox + col * res)
            for j, ch in enumerate(t):
                if k + j < width:
                    lab[k + j] = ch
        k += 1
    print('          ' + ''.join(lab))
    print('          (x / m)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
