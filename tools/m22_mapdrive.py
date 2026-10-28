# -*- coding: utf-8 -*-
"""
m22_mapdrive.py —— 建图行驶：朝最空的方向小步走，边走边扩图

## 为什么不用 m6_scan

`m6_scan` 每轮转满 360° 后只检查**正前方**能不能走。
结果它永远面向同一个方向判断，能走就每圈挪 0.15 m，不能走就永远原地转 ——
实测 5 圈 50 次旋转、**0 次前进**，地图只有一个视点，没有视差，建不出完整地图。

## 这个脚本的做法

每一轮：
  1. 扫描 12 个方位，找**最空**的那个方向
  2. 转到它（原地旋转，安全）
  3. 若最空方向净空 >= MOVE_MIN_CLEAR，前进一小步
  4. 每走 N 步，补做一次 360° 慢转（把新位置四周吃进来）

这样机器人会自己往空间大的地方走，逐步扩图。

## 安全

  全部动作走 `realbot.py` 的护栏：
    · MIN_VALID = 0.55   近距点当车体自遮挡丢掉
    · FRONT_STOP = 0.45  前方 < 0.45 m 急停（车半径 0.28 + 0.17 余量）
    · 看门狗 0.6 s 兜底（底盘没有 cmd_vel 超时）
  速度 0.09 m/s，步长 0.15 m。
"""
from __future__ import print_function

import math
import sys
import time

import rospy
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')
from realbot import RealBot, FRONT_STOP, MIN_VALID   # noqa: E402

MOVE_MIN_CLEAR = 0.55      # m  最空方向至少这么空才前进
MOVE_STEP = 0.15           # m
V = 0.09                   # m/s
FULL_TURN_EVERY = 4        # 每走这么多步，补一次 360° 慢转
TURN_STEP = 45.0           # deg 找方向时每次转多少


class MapWatch(object):
    def __init__(self):
        self.map = None
        rospy.Subscriber('/map', OccupancyGrid, self._cb, queue_size=1)

    def _cb(self, m):
        self.map = m

    def stats(self):
        if self.map is None:
            return None
        d = self.map.data
        free = sum(1 for v in d if v == 0)
        occ = sum(1 for v in d if v > 50)
        return (free, occ, 100.0 * (free + occ) / len(d))


def best_dir(d, half_deg=22.0):
    """返回 (角度, 净空) —— 12 个方位里最空的那个。"""
    best = (0.0, -1.0)
    for a in range(0, 360, 30):
        r = d.sector_min_robust(math.radians(a), half_deg=half_deg)
        if r and r > best[1]:
            best = (float(a), r)
    return best


def full_turn(d, mw, label=''):
    print('      补 360° 慢转 %s' % label)
    for _ in range(8):
        d.rotate(TURN_STEP, timeout=25.0, tol_deg=2.0)
        time.sleep(1.2)


def main():
    minutes = 4.0
    for i, a in enumerate(sys.argv):
        if a == '--minutes' and i + 1 < len(sys.argv):
            minutes = float(sys.argv[i + 1])

    rospy.init_node('m22_mapdrive', anonymous=True)
    d = RealBot(verbose=False)
    mw = MapWatch()

    print('')
    print('=' * 66)
    print('  建图行驶：朝最空方向小步走（%.1f 分钟）' % minutes)
    print('=' * 66)
    print('  MIN_VALID=%.2f  FRONT_STOP=%.2f  MOVE_MIN=%.2f  步长=%.2f'
          % (MIN_VALID, FRONT_STOP, MOVE_MIN_CLEAR, MOVE_STEP))

    if not d.ready(timeout=20):
        print('  [FAIL] 传感器没起来')
        return 1
    t0 = time.time()
    while mw.stats() is None and time.time() - t0 < 20:
        time.sleep(0.5)
    print('  起始地图: %s' % (mw.stats(),))

    print('  先做一次 360° 慢转把四周吃进来')
    full_turn(d, mw, '(初始)')
    print('  转完: %s' % (mw.stats(),))

    t_end = time.time() + minutes * 60.0
    steps = 0
    turns = 0
    try:
        while time.time() < t_end and not rospy.is_shutdown():
            ang, clear = best_dir(d)
            el = time.time() - t0
            print('  [%5.0fs] 最空方向 %3.0f°（%.2f m）'
                  % (el, ang, clear))
            if clear < MOVE_MIN_CLEAR:
                print('            最空也只有 %.2f m < %.2f → 原地小转找别的方向'
                      % (clear, MOVE_MIN_CLEAR))
                d.rotate(60.0, timeout=25.0, tol_deg=2.0)
                turns += 1
                time.sleep(1.2)
                continue

            # 转到最空方向
            d.rotate(ang, timeout=30.0, tol_deg=3.0)
            turns += 1
            time.sleep(1.0)

            # 转过去之后再确认一次前方（旋转后前方就是刚才那个方位）
            fm = d.sector_min_robust(0.0, half_deg=22.0)
            if fm is None or fm < MOVE_MIN_CLEAR:
                print('            转到后前方 %.2f m 不够，跳过本次前进'
                      % (fm if fm else -1))
                continue

            print('            前进 %.2f m（前方 %.2f m）' % (MOVE_STEP, fm))
            d.forward(MOVE_STEP, speed=V)
            steps += 1
            time.sleep(0.8)
            print('            地图: %s' % (mw.stats(),))

            if steps % FULL_TURN_EVERY == 0:
                full_turn(d, mw, '(第 %d 步后)' % steps)
                print('            地图: %s' % (mw.stats(),))

    except KeyboardInterrupt:
        print('  [中断]')
    finally:
        d.stop('建图行驶结束')

    print('')
    print('=' * 66)
    print('  结束：%d 步前进   %d 次转向   用时 %.0f s'
          % (steps, turns, time.time() - t0))
    print('  最终地图: %s' % (mw.stats(),))
    print('=' * 66)
    return 0


if __name__ == '__main__':
    sys.exit(main())
