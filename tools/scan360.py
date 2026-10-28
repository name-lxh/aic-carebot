# -*- coding: utf-8 -*-
"""
scan360.py —— 原地转一圈建极坐标地图

## 做什么

  1. 用 IMU 闭环，把机器人原地转 360°（分 24 步，每步 15°）
  2. 每步采集一帧 /scan
  3. 把每帧的点按**当时的 IMU 绝对航向**投影到同一个世界坐标系
  4. 输出 ASCII 地图 + 各方向的墙距离

## 为什么这样能定位

激光点 (r, θ_laser) 在**车体系**里；车体在世界里的朝向是 IMU 给的 Φ。
所以点的**世界方位角** = θ_laser + Φ。
把全程的点都换算到同一个世界方位角上，
就得到一张**以机器人当前位置为原点、朝向为绝对方向**的极坐标地图。

⚠️ 这不能给出"机器人在场地的哪个绝对坐标"（那需要已知参考点），
   但能给出：**四面墙/隔断在哪个方位、多远**，
   足够用来规划"往哪走"。

## 安全

  · 只旋转，不前进 —— 不改变机器人与墙的距离，不会撞
  · 速度 0.25 rad/s（很慢）
  · 每步都有超时
  · 看门狗兜底
"""
from __future__ import print_function

import json
import math
import sys
import time

import rospy

sys.path.insert(0, '/tmp')
from realbot import RealBot, quat_to_yaw, norm_angle   # noqa: E402

STEP_DEG = 15.0
STEPS = 24


def laser_to_world(scan, yaw_abs):
    """把一帧激光转成世界坐标点列表 [(x, y, r, bearing_deg)]。"""
    out = []
    if scan is None:
        return out
    for i, v in enumerate(scan.ranges):
        if v != v or v == float('inf') or v <= 0.05 or v > 8.0:
            continue
        a = scan.angle_min + i * scan.angle_increment
        bear = norm_angle(a + yaw_abs)          # 世界方位角
        out.append((v * math.cos(bear), v * math.sin(bear),
                    v, math.degrees(bear)))
    return out


def main():
    rospy.init_node('scan360', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv))

    print('')
    print('=' * 64)
    print('  原地 360° 扫描建图')
    print('=' * 64)

    if not d.ready(timeout=20):
        print('  ✘ /scan 或 /imu_data 收不到')
        return 1

    yaw_start = d.yaw()
    print('  起始 IMU yaw = %.2f°' % math.degrees(yaw_start))
    allpts = []

    # 先把当前位置那帧收进去
    time.sleep(0.5)
    allpts += laser_to_world(d._scan, d.yaw())

    print('')
    print('  开始旋转（%d 步 × %.0f°）' % (STEPS, STEP_DEG))
    print('  进度：', end='')
    for k in range(STEPS):
        if rospy.is_shutdown():
            break
        ok = d.rotate(STEP_DEG, timeout=25.0, tol_deg=3.0)
        time.sleep(0.35)
        y = d.yaw()
        npts = len(laser_to_world(d._scan, y))
        allpts += laser_to_world(d._scan, y)
        sys.stdout.write('.')
        sys.stdout.flush()

    print('')
    d.stop('扫描完成')
    yaw_end = d.yaw()
    total = math.degrees(norm_angle(yaw_end - yaw_start))
    print('  转完一圈：实际共转了 %.1f°（目标 360°）' % total)
    print('  累计点数 %d' % len(allpts))

    # ---- 按 5° 分桶，取每桶最小距离 ----
    print('')
    print('---- 世界方位 → 最近墙距离（每 5°）----')
    buckets = {}
    for _x, _y, r, b in allpts:
        k = int((b + 360) % 360 // 5) * 5
        if k not in buckets or r < buckets[k]:
            buckets[k] = r
    for k in range(0, 360, 5):
        v = buckets.get(k)
        if v is None:
            print('    %3d°   ---' % k)
            continue
        bar = '#' * min(50, int(v * 12))
        print('    %3d°  %6.3f m  %s' % (k, v, bar))

    # ---- ASCII 地图 ----
    print('')
    print('---- ASCII 地图（机器人 = R，每格 10cm，前 12m 范围）----')
    SIZE = 60           # -3.0m .. +3.0m
    RES = 0.10
    grid = [[' '] * SIZE for _ in range(SIZE)]
    for x, y, _r, _b in allpts:
        gx = int(x / RES) + SIZE // 2
        gy = int(y / RES) + SIZE // 2
        if 0 <= gx < SIZE and 0 <= gy < SIZE:
            grid[gy][gx] = '#'
    grid[SIZE // 2][SIZE // 2] = 'R'
    for row in range(SIZE - 1, -1, -1):
        print('    |' + ''.join(grid[row]) + '|')
    print('    (上=世界 +y / IMU 角度 90 度，右=+x / 0 度；R=机器人)')
    print('    (每个 # 是 10cm 格，可能有重叠与噪声)')

    # ---- 存盘 ----
    try:
        with open('/tmp/scan360.json', 'w') as fp:
            json.dump({'yaw_start': yaw_start,
                       'points': [(round(x, 3), round(y, 3))
                                  for x, y, _r, _b in allpts]}, fp)
        print('  已存 /tmp/scan360.json（%d 点）' % len(allpts))
    except Exception as e:
        print('  存盘失败: %r' % e)
    print('=' * 64)
    return 0


if __name__ == '__main__':
    sys.exit(main())
