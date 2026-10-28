# -*- coding: utf-8 -*-
"""详细分析一帧 /scan，用来：
   1. 判断近处那些 0.1m 的回波是不是**扫到自己车体**
   2. 把场地几何量出来（各方向到墙的距离）
   3. 给出"最空的方向"，供第一次移动时选安全方向

⚠️ 写成文件而不是 heredoc —— Py2 下 heredoc 里的中文注释会
   因缺编码声明而 SyntaxError。
"""
from __future__ import print_function

import math

import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('scan_detail', anonymous=True)
    frames = []
    for _ in range(5):
        try:
            frames.append(rospy.wait_for_message('/scan', LaserScan,
                                                 timeout=10))
        except Exception:
            break
    if not frames:
        print('读不到 /scan')
        return 1
    s = frames[-1]
    n = len(s.ranges)
    print('帧数=%d  点数=%d  frame=%s' % (len(frames), n, s.header.frame_id))

    def ang(i):
        return math.degrees(s.angle_min + i * s.angle_increment)

    # ---- 1. 近处回波（怀疑自扫）----
    print('')
    print('===== 1. 近处回波（< 0.30 m）=====')
    near = []
    for i, v in enumerate(s.ranges):
        if v == v and v != float('inf') and 0 < v < 0.30:
            near.append((v, ang(i)))
    print('  数量 %d / %d' % (len(near), n))
    if near:
        near.sort()
        print('  最近的 10 个（距离, 角度°）：')
        for v, a in near[:10]:
            print('      %.3f m   %7.1f°' % (v, a))
        # 角度分布：自扫的话会集中在某个角度范围
        angs = sorted(a for _v, a in near)
        print('  角度范围：%.1f° ~ %.1f°' % (angs[0], angs[-1]))
        # 按 30° 分桶看集中度
        buckets = {}
        for _v, a in near:
            b = int((a + 360) % 360 // 30) * 30
            buckets[b] = buckets.get(b, 0) + 1
        print('  按 30° 分桶：%s'
              % ', '.join('%d°:%d' % (k, v)
                          for k, v in sorted(buckets.items())))
        print('  ⚠️ 若集中在**少数几个角度**→ 像自扫；'
              '若**均匀分布**→ 更像地面/噪声')

    # ---- 2. 逐 10° 的距离剖面（看场地形状）----
    print('')
    print('===== 2. 逐 10° 的距离剖面（最近/中位）=====')
    step = int(round(math.radians(10) / s.angle_increment))
    print('  角度范围      最近      中位      点的个数')
    for b in range(0, n, step):
        seg = [v for v in s.ranges[b:b + step]
               if v == v and v != float('inf') and v > 0.01]
        a0 = ang(b)
        a1 = ang(min(b + step, n - 1))
        if not seg:
            print('  %6.1f-%6.1f      -         -         0' % (a0, a1))
            continue
        ss = sorted(seg)
        print('  %6.1f-%6.1f  %7.3f  %7.3f   %5d'
              % (a0, a1, ss[0], ss[len(ss) // 2], len(seg)))

    # ---- 3. 最空的方向（>1.2 m 的连续扇区）----
    print('')
    print('===== 3. 各方向可用空间（用于选安全的移动方向）=====')
    print('  找出连续 >=1.2 m 的角度区间：')
    good = [i for i, v in enumerate(s.ranges)
            if v == v and v != float('inf') and v >= 1.2]
    if not good:
        print('      ★ 没有任何方向有 1.2 m 以上空间 —— 机器人被堵住了？')
    else:
        runs = []
        start = good[0]
        prev = good[0]
        for i in good[1:]:
            if i == prev + 1:
                prev = i
            else:
                runs.append((start, prev))
                start = prev = i
        runs.append((start, prev))
        # 合并首尾（环形）
        if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == n - 1:
            a, b = runs.pop()
            runs[0] = (a - n, runs[0][1])
        runs.sort(key=lambda r: r[1] - r[0], reverse=True)
        for a, b in runs[:6]:
            print('      %7.1f° ~ %7.1f°   宽 %5.1f°'
                  % (ang(a % n), ang(b % n),
                     (b - a) * math.degrees(s.angle_increment)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
