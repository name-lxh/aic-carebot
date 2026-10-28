# -*- coding: utf-8 -*-
"""
survey.py —— 场地几何普查（**不发任何速度**）

目的：把机器人周围的墙/隔断位置量清楚，这样：
  1. 能判断当前朝向是不是被挡住了
  2. 能推出机器人在场地里的**大致位置**（用已知的场地尺寸反推）
  3. 能给下一步选一条**安全的移动方向**

场地已知尺寸（来自赛题图二）：
    总 3000 × 3000 mm
    客厅在下方 y ∈ [0, 1200]，三个房间在上方 y ∈ [1200, 3000]
    房间宽各 1000，从左到右：D 卧室 | C 厨房 | B 书房 （注：图上 D 在左）
"""
from __future__ import print_function

import math
import sys
import time

import rospy
from sensor_msgs.msg import LaserScan

sys.path.insert(0, '/tmp')
from realbot import quat_to_yaw   # noqa: E402


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    rospy.init_node('survey', anonymous=True)
    try:
        s = rospy.wait_for_message('/scan', LaserScan, timeout=15)
    except Exception as e:
        print('读不到 /scan: %r' % e)
        return 1
    try:
        imu = rospy.wait_for_message('/imu_data', __import__(
            'sensor_msgs.msg', fromlist=['Imu']).Imu, timeout=10)
        yaw = quat_to_yaw(imu.orientation)
    except Exception:
        yaw = None

    n = len(s.ranges)
    inc = s.angle_increment

    print('')
    print('=' * 64)
    print('  场地几何普查（只读，不发速度）')
    print('=' * 64)
    print('  IMU yaw = %s' % (('%.2f°' % math.degrees(yaw))
                              if yaw is not None else '（读不到）'))
    print('  点数 %d  角度增量 %.4f°' % (n, math.degrees(inc)))

    def dist_at(a_deg):
        """给定方向（激光坐标系，0=车头）取 ±2° 内最小值。"""
        half = math.radians(2.0)
        best = None
        for i, v in enumerate(s.ranges):
            if v != v or v == float('inf') or v <= 0.02:
                continue
            a = s.angle_min + i * inc
            if abs(norm(a - math.radians(a_deg))) <= half:
                if best is None or v < best:
                    best = v
        return best

    # ---- 1. 每 15° 一个剖面 ----
    print('')
    print('---- 每 15° 的最近距离（激光坐标系，0°=车头，逆时针为正）----')
    rows = []
    for d in range(0, 360, 15):
        v = dist_at(d)
        rows.append((d, v))
        bar = ''
        if v is not None:
            bar = '#' * min(40, int(v * 10))
        print('    %5d°  %s  %s'
              % (d, ('%6.3f m' % v) if v is not None else '   ---  ',
                 bar))

    # ---- 2. 找"最空的方向"（>=1.5m 连续区间）----
    print('')
    print('---- 可用空间 >=1.5 m 的连续方向区间 ----')
    good = [i for i, v in enumerate(s.ranges)
            if v == v and v != float('inf') and v >= 1.5]
    if not good:
        print('    ★ 没有任何方向有 1.5 m 以上净空')
    else:
        runs = []
        st = prev = good[0]
        for i in good[1:]:
            if i == prev + 1:
                prev = i
            else:
                runs.append((st, prev))
                st = prev = i
        runs.append((st, prev))
        if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == n - 1:
            a, b = runs.pop()
            runs[0] = (a - n, runs[0][1])
        runs.sort(key=lambda r: r[1] - r[0], reverse=True)
        for a, b in runs[:5]:
            a0 = math.degrees(s.angle_min + (a % n) * inc)
            a1 = math.degrees(s.angle_min + (b % n) * inc)
            print('    %7.1f° ~ %7.1f°   宽 %5.1f°'
                  % (a0, a1, (b - a) * math.degrees(inc)))

    # ---- 3. 用场地尺寸反推位置（假设能看到外墙）----
    print('')
    print('---- 反推机器人位置（做一个粗略估计）----')
    print('    假设 0°/90°/180°/270° 里至少有一个打在外墙上：')
    for d, nm in ((0, '前'), (90, '左'), (180, '后'), (270, '右')):
        v = dist_at(d)
        print('      %s(%3d°)  %s' % (nm, d,
              ('%.3f m' % v) if v is not None else '---'))

    print('')
    print('  ⚠️ 这只是粗估：场地里还有**内部隔墙**，')
    print('     某个方向测到的可能不是外墙而是隔断。')
    print('     要精确定位需要建图，或人工把机器人摆到已知位置。')
    print('=' * 64)
    return 0


if __name__ == '__main__':
    sys.exit(main())
