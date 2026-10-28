# -*- coding: utf-8 -*-
"""
m6_scan.py —— 原地慢转 + 小步前进的 5 分钟扫描建图

## 场景

小车撞墙把墙推动了，之后被**手动搬到空旷位置**（四周分别是 B/C/D/A 四个房间）。
场地已变、机器人被搬动 → 已重开新图（m6_fresh_and_scan.sh）。

## 动作设计

**主要是"原地慢转 360°"，前进只是附带。**

理由：
  · 旋转**不改变与任何墙的距离**，只需要车体半径（0.25 m）的余量，
    在空旷位置是**最安全的动作**
  · gmapping 全靠扫描匹配，**同一个位置转一圈就能把 360° 的墙都扫进来**
  · 前进只在"前方净空明显足够"时做**一小步**，用来补视差、看进房间门洞

每一轮：
  1. 分 12 步慢慢转满 360°（每步 30°，转完停 1.5 s 让 gmapping 处理）
  2. 若前方净空 >= 0.90 m → 前进 0.20 m，否则不动
  3. 重复，直到时间用完

## 安全

  · 只用 realbot.py 的护栏（MIN_VALID=0.55 / FRONT_STOP=0.70）
  · 旋转本身不发线速度
  · 看门狗兜底（底盘没有 cmd_vel 超时归零）
"""
from __future__ import print_function

import math
import sys
import time

import rospy
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')
from realbot import RealBot, FRONT_STOP, MIN_VALID   # noqa: E402

TURN_STEP = 30.0        # deg  每次转多少
SETTLE = 1.5            # s    转完停多久让 gmapping 吃数据
MOVE_MIN_CLEAR = 0.55   # m    前方净空至少这么大才前进
                        #  原 0.90 在 3m 场地里几乎永远不满足（实测 0 次前进），
                        #  导致地图只有一个视点、没有视差，建不出完整地图。
                        #  0.55 = 车半径 0.28 + 0.27 余量。
MOVE_STEP = 0.15        # m    小步前进，方便慢慢扩图
V = 0.09                # m/s


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


def main():
    minutes = 5.0
    for i, a in enumerate(sys.argv):
        if a == '--minutes' and i + 1 < len(sys.argv):
            minutes = float(sys.argv[i + 1])

    rospy.init_node('m6_scan', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv), verbose=False)
    mw = MapWatch()

    print('')
    print('=' * 66)
    print('  慢转 360° + 小步前进 —— 扫描建图（%.1f 分钟）' % minutes)
    print('=' * 66)
    print('  MIN_VALID=%.2f  FRONT_STOP=%.2f' % (MIN_VALID, FRONT_STOP))

    if not d.ready(timeout=20):
        print('  [FAIL] /scan 或 /imu_data 收不到')
        return 1

    t0 = time.time()
    while mw.stats() is None and time.time() - t0 < 20:
        time.sleep(0.5)
    print('  初始地图: %s' % (mw.stats(),))

    # ---- 先报一下四周净空，确认这个位置适合扫 ----
    print('')
    print('  四周净空（12 个方位）:')
    clears = []
    for a in range(0, 360, 30):
        r = d.sector_min_robust(math.radians(a), half_deg=20.0)
        clears.append((a, r))
        print('    %3d°  %s' % (a, ('%.2f m' % r) if r else 'None'))
    if clears:
        mx = max([c[1] for c in clears if c[1]] or [0])
        print('  最大净空 %.2f m' % mx)
        if mx < MOVE_MIN_CLEAR:
            print('  [!] 最大净空不足 %.2f m —— 只做原地旋转，不前进'
                  % MOVE_MIN_CLEAR)

    print('')
    print('  ⚠️ 开始扫描')
    for i in (5, 4, 3, 2, 1):
        sys.stdout.write('     %d...\r' % i)
        sys.stdout.flush()
        time.sleep(1.0)
    print('')

    t_end = time.time() + minutes * 60.0
    n_turn = 0
    n_move = 0
    laps = 0

    try:
        while time.time() < t_end and not rospy.is_shutdown():
            laps += 1
            el = time.time() - t0
            print('  [%5.0fs] 第 %d 圈：原地慢转 360°' % (el, laps))
            for k in range(int(360 / TURN_STEP)):
                if time.time() > t_end:
                    break
                d.rotate(TURN_STEP, timeout=25.0, tol_deg=2.0)
                n_turn += 1
                time.sleep(SETTLE)
            st = mw.stats()
            print('            转完一圈  地图: %s' % (st,))

            # 一圈转完，看要不要挪一小步
            if time.time() >= t_end:
                break
            fm = d.sector_min_robust(0.0)
            if fm is not None and fm >= MOVE_MIN_CLEAR:
                print('            前方 %.2f m 够空 → 前进 %.2f m'
                      % (fm, MOVE_STEP))
                d.forward(MOVE_STEP, speed=V)
                time.sleep(1.0)
                n_move += 1
                st = mw.stats()
                print('            前进后  地图: %s' % (st,))
            else:
                cur = math.degrees(d.yaw())
                best = (0.0, 0.0)
                for a in range(0, 360, 30):
                    r = d.sector_min_robust(math.radians(a), half_deg=20.0)
                    if r and r > best[1]:
                        best = (a, r)
                print('            前方 %s 不够空 → 不前进（最空方向 %.0f° %.2f m）'
                      % (('%.2f m' % fm) if fm else 'None', best[0], best[1]))

    except KeyboardInterrupt:
        print('  [中断]')
    finally:
        d.stop('扫描结束')

    print('')
    print('=' * 66)
    print('  扫描结束，用时 %.0f s' % (time.time() - t0))
    print('    转 %d 次   前进 %d 次   共 %d 圈' % (n_turn, n_move, laps))
    print('    最终地图: %s' % (mw.stats(),))
    print('=' * 66)
    return 0


if __name__ == '__main__':
    sys.exit(main())
