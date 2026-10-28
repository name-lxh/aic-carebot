# -*- coding: utf-8 -*-
"""
analyze360.py —— 重新分析 scan360.json（在机器人本地跑）

## 为什么要重算

`scan360.py` 只过滤了 `< 0.05 m`，而那批 **0.10 m 的近距回波**
在**所有方位**都出现 —— 那是**传感器自身的近距噪声**，不是墙。
它们把"每 5° 取最近"的结果压成了 0.10 m，把真实几何全盖住了。

## 怎么判断它是噪声

  1. 360° 每个方位都有 0.10 m 的回波 —— 现实里不可能（机器人
     不可能同时贴着四面墙）
  2. 场地是 3×3，机器人在客厅中间，离墙至少 0.6 m
  3. 高度也对不上：雷达在车顶，自扫应该来自车体正上方/正下方

## 处理

  · 用**截断下限**（默认 0.30 m）把近噪滤掉
  · 同时输出**不截断**的统计，方便对比确认
  · 给出每个方位的**中位数**（比"最小值"抗噪）
"""
from __future__ import print_function

import json
import math
import sys


def main():
    CUT = 0.30
    for a in sys.argv[1:]:
        if a.startswith('--cut='):
            CUT = float(a.split('=')[1])

    try:
        with open('/tmp/scan360.json') as fp:
            data = json.load(fp)
    except Exception as e:
        print('读不到 /tmp/scan360.json: %r' % e)
        return 1

    pts = data.get('points') or []
    print('总点数 %d' % len(pts))

    # ---- 1. 先看距离分布，确认近噪 ----
    rs = sorted(math.hypot(x, y) for x, y in pts)
    print('')
    print('---- 距离分布（判断近噪）----')
    for p in (0, 1, 5, 10, 25, 50, 75, 90, 95, 99, 100):
        i = min(len(rs) - 1, int(len(rs) * p / 100.0))
        print('    %3d%%  %.3f m' % (p, rs[i]))

    # 按 0.05 m 分桶统计，看那批近点有多集中
    print('')
    print('---- 距离直方图（0~1.2 m，每 0.05 m 一桶）----')
    hist = {}
    for r in rs:
        k = int(r / 0.05)
        hist[k] = hist.get(k, 0) + 1
    for k in range(0, 24):
        c = hist.get(k, 0)
        if not c:
            continue
        bar = '#' * min(60, int(c * 60.0 / max(1, max(hist.values()))))
        print('    %.2f~%.2f m  %6d  %s' % (k * 0.05, (k + 1) * 0.05, c, bar))

    # ---- 2. 用截断下限重算方位剖面 ----
    kept = [(x, y) for x, y in pts if math.hypot(x, y) >= CUT]
    print('')
    print('截断下限 %.2f m：保留 %d / %d 点' % (CUT, len(kept), len(pts)))

    if not kept:
        print('★ 截断后没有点了 —— 下限设太高？')
        return 1

    # 每 10° 取**中位数**
    buckets = {}
    for x, y in kept:
        r = math.hypot(x, y)
        b = math.degrees(math.atan2(y, x))
        k = int((b + 360) % 360 // 10) * 10
        buckets.setdefault(k, []).append(r)

    print('')
    print('---- 世界方位 → 距离（每 10°，给最小值/中位数/点数）----')
    for k in range(0, 360, 10):
        v = buckets.get(k)
        if not v:
            print('    %3d°   ---' % k)
            continue
        v.sort()
        med = v[len(v) // 2]
        bar = '#' * min(45, int(med * 12))
        print('    %3d°  min %6.3f  med %6.3f  n=%4d  %s'
              % (k, v[0], med, len(v), bar))

    # ---- 3. ASCII 地图（用中位数过滤孤立噪点）----
    print('')
    print('---- ASCII 地图（每格 10cm，R=机器人）----')
    SIZE = 60
    RES = 0.10
    grid = [[' '] * SIZE for _ in range(SIZE)]
    for x, y in kept:
        gx = int(x / RES) + SIZE // 2
        gy = int(y / RES) + SIZE // 2
        if 0 <= gx < SIZE and 0 <= gy < SIZE:
            grid[gy][gx] = '#'
    grid[SIZE // 2][SIZE // 2] = 'R'
    for row in range(SIZE - 1, -1, -1):
        line = ''.join(grid[row])
        if line.strip():
            print('    |%s|' % line)
    print('    (上=+y / 90 度，右=+x / 0 度)')

    # ---- 4. 找最空的方向 ----
    print('')
    print('---- 净空 >= %.1f m 的连续方向区间 ----' % 1.0)
    angs = []
    for k in range(0, 360, 5):
        vs = []
        for x, y in kept:
            r = math.hypot(x, y)
            b = math.degrees(math.atan2(y, x))
            if int((b + 360) % 360 // 5) * 5 == k:
                vs.append(r)
        angs.append((k, min(vs) if vs else 99.0))
    runs = []
    st = None
    for k, v in angs + [(360, 0.0)]:
        if v >= 1.0:
            if st is None:
                st = k
        else:
            if st is not None:
                runs.append((st, k - 5))
                st = None
    if not runs:
        print('    ★ 没有 >=1.0 m 的连续净空')
    for a, b in runs:
        print('    %3d° ~ %3d°  宽 %3d°' % (a, b, (b - a) % 360 + 5))
    return 0


if __name__ == '__main__':
    sys.exit(main())
