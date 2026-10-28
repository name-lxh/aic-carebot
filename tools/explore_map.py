# -*- coding: utf-8 -*-
"""
explore_map.py —— 自动探索建图

## 策略（保守版）

反复做：
  1. 看当前朝向前方够不够空（>= `AHEAD_MIN`）
  2. 够 → 前进一小步（`STEP`）
  3. 不够 → 转向空间更大的一侧（`TURN_STEP`）
  4. 每步都存一次地图（`/map` 订阅）
  5. 到达时间预算就停

⚠️ 这个策略**不追求覆盖率最优**，只追求"安全地把场地走一遍"。
   3x3 m 的场地，走几十步基本能覆盖。

## 安全（与本项目其它脚本一致）

  · 前向激光护栏（< `FRONT_STOP` 立即停）
  · 看门狗（底盘没有 cmd_vel 超时归零）
  · 时间预算 + 每步硬超时
  · Ctrl-C / 异常都走统一 stop()

## 用法

    python explore_map.py --minutes 8
"""
from __future__ import print_function

import math
import sys
import time

import rospy
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, '/tmp')
from realbot import RealBot          # noqa: E402

AHEAD_MIN = 0.70        # m  前方至少这么空才敢前进
                        # （客厅只有 1.2 m 深，0.85 太严会一步都走不了）
STEP = 0.25             # m  每步前进距离
TURN_STEP = 45.0        # deg 被挡住时转多少
SIDE_LOOK = 60.0        # deg 转向时看多宽


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
        return (self.map.info.width, self.map.info.height,
                self.map.info.resolution, free, occ)


def pick_turn(d):
    """前方被挡时，比较左右哪边更空，返回该转的角度。"""
    left = d.sector_min_robust(math.radians(90), half_deg=SIDE_LOOK / 2)
    right = d.sector_min_robust(math.radians(-90), half_deg=SIDE_LOOK / 2)
    lv = left if left is not None else 0.0
    rv = right if right is not None else 0.0
    if lv >= rv:
        return TURN_STEP, lv, rv
    return -TURN_STEP, lv, rv


def main():
    minutes = 8.0
    for i, a in enumerate(sys.argv):
        if a == '--minutes' and i + 1 < len(sys.argv):
            minutes = float(sys.argv[i + 1])

    rospy.init_node('explore_map', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv))
    mw = MapWatch()

    print('')
    print('=' * 64)
    print('  自动探索建图（预算 %.1f 分钟）' % minutes)
    print('=' * 64)

    if not d.ready(timeout=20):
        print('  ✘ /scan 或 /imu_data 收不到')
        return 1

    st = mw.stats()
    print('  初始地图: %s' % (st,))
    print('  起始 yaw = %.1f°' % math.degrees(d.yaw()))
    print('')
    print('  策略：前方 >= %.2f m 就前进 %.2f m，否则转向 %.0f°'
          % (AHEAD_MIN, STEP, TURN_STEP))
    print('  ⚠️ 机器人即将移动 —— 请确认场地里没人')
    for i in (5, 4, 3, 2, 1):
        sys.stdout.write('     %d...\r' % i)
        sys.stdout.flush()
        time.sleep(1.0)
    print('')

    t_end = time.time() + minutes * 60.0
    n_fwd = 0
    n_turn = 0
    n_abort = 0
    t0 = time.time()

    try:
        while time.time() < t_end and not rospy.is_shutdown():
            el = time.time() - t0
            ahead = d.sector_min_robust(0.0)
            av = ahead if ahead is not None else 0.0

            if av >= AHEAD_MIN:
                if d.forward(STEP, speed=0.10):
                    n_fwd += 1
                else:
                    n_abort += 1
                    ang, lv, rv = pick_turn(d)
                    print('  [%5.0fs] 被挡(前 %.2f 左 %.2f 右 %.2f) → 转 %+.0f°'
                          % (el, av, lv, rv, ang))
                    d.rotate(ang, timeout=25.0)
                    n_turn += 1
            else:
                ang, lv, rv = pick_turn(d)
                print('  [%5.0fs] 前方仅 %.2f m(左 %.2f 右 %.2f) → 转 %+.0f°'
                      % (el, av, lv, rv, ang))
                d.rotate(ang, timeout=25.0)
                n_turn += 1

            if (n_fwd + n_turn) % 4 == 0:
                st = mw.stats()
                if st:
                    print('  [%5.0fs] 地图 %dx%d@%.2f  空闲 %d  占用 %d'
                          % (el, st[0], st[1], st[2], st[3], st[4]))
    except KeyboardInterrupt:
        print('  [中断]')
    finally:
        d.stop('探索结束')

    st = mw.stats()
    print('')
    print('=' * 64)
    print('  探索结束：前进 %d 次，转向 %d 次，被拦 %d 次'
          % (n_fwd, n_turn, n_abort))
    print('  最终地图: %s' % (st,))
    print('  yaw = %.1f°' % math.degrees(d.yaw()))
    print('=' * 64)
    return 0


if __name__ == '__main__':
    sys.exit(main())
