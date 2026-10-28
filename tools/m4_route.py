# -*- coding: utf-8 -*-
"""
m4_route.py —— 按已知场地轴向做确定性扫描建图

## 为什么不用反应式探索

`explore_map.py` 在这台车上**走不动**：客厅只有 1.2 m 深，
减去车体自身遮挡 ~0.35 m，可用空间只剩 ~0.85 m，
而它每步要 0.25 m、门槛 0.70 m，于是不停在 0.37~0.51 m 之间来回摆：

    [54s] 前方仅 0.42 m(左 0.82 右 1.90) → 转 -45°
    [57s] 前方仅 0.50 m(左 1.47 右 0.93) → 转 +45°
    [61s] 前方仅 0.38 m(左 0.82 右 1.61) → 转 -45°   ...

## 这里的做法

既然已经量出机器人在**客厅正中**、并推出了四个方向的绝对朝向，
就直接按航线走，不再"猜下一步往哪"。

### 场地轴向怎么来的

初始 survey（车头朝南）：
    前(0°)  0.630 m     左(90°)  1.518 m     右(270°) 1.540 m
对照图纸（客厅 3000×1200，机器人在正中 → 北/南各 0.6 m、东/西各 1.5 m）
→ 完全吻合，且当时 **IMU yaw = 21.6°**，即"朝南"对应 21.6°。

于是（ROS yaw 逆时针为正，+90° 是从南转到东）：
    南 = 21.6°   东 = 111.6°   西 = -68.4°   北 = -158.4°

### 航线

    1. 朝东 → 走到东墙
    2. 掉头朝西 → 走到西墙（贯穿整个客厅长轴 3 m）
    3. 朝北 → 靠近客厅/房间之间的隔墙，找门洞
    4. 朝南 → 回到客厅

每段都：小步前进 + 前向护栏 + 到边界就停，**不硬冲**。
"""
from __future__ import print_function

import math
import sys
import time

import rospy
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')
from realbot import RealBot, FRONT_STOP, MIN_VALID   # noqa: E402


# 场地轴向（绝对 IMU yaw，单位度）
YAW_SOUTH = 21.6
YAW_EAST = YAW_SOUTH + 90.0      # 111.6
YAW_WEST = YAW_SOUTH - 90.0      # -68.4
YAW_NORTH = YAW_SOUTH + 180.0    # 201.6 -> 归一化 -158.4

STEP = 0.20          # m  每小步
V = 0.10             # m/s
# 前进时如果前方小于这个值就停（比 FRONT_STOP 略宽一点，留反应余量）
STOP_MARGIN = FRONT_STOP + 0.08


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
        return (len(d), free, occ, 100.0 * (free + occ) / len(d))


def norm180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def face(d, target_deg, tol=3.0):
    """转到绝对朝向 target_deg（度）。"""
    cur = math.degrees(d.yaw())
    delta = norm180(target_deg - cur)
    print('    转向: 当前 %.1f° → 目标 %.1f°（相对 %+.1f°）'
          % (cur, target_deg, delta))
    d.rotate(delta, timeout=40.0, tol_deg=tol)
    return math.degrees(d.yaw())


def walk(d, max_m, label=''):
    """朝当前车头小步前进，直到边界/护栏/预算。"""
    done = 0.0
    while done < max_m and not rospy.is_shutdown():
        fm = d.sector_min_robust(0.0)
        if fm is None:
            print('    %s 前方无有效点，停' % label)
            break
        if fm < STOP_MARGIN:
            print('    %s 前方 %.2f m < %.2f m，停（走了 %.2f m）'
                  % (label, fm, STOP_MARGIN, done))
            break
        # 一步不要超过剩余预算
        step = min(STEP, max_m - done)
        # 也不要撞上去：留 STOP_MARGIN 余量
        step = min(step, max(0.02, fm - STOP_MARGIN))
        d.forward(step, speed=V)
        done += step
    return done


def main():
    minutes = 6.0
    for i, a in enumerate(sys.argv):
        if a == '--minutes' and i + 1 < len(sys.argv):
            minutes = float(sys.argv[i + 1])

    rospy.init_node('m4_route', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv), verbose=False)
    mw = MapWatch()

    print('')
    print('=' * 66)
    print('  确定性航线扫描建图')
    print('=' * 66)
    print('  FRONT_STOP=%.2f  MIN_VALID=%.2f  停进阈值=%.2f'
          % (FRONT_STOP, MIN_VALID, STOP_MARGIN))

    if not d.ready(timeout=20):
        print('  [FAIL] /scan 或 /imu_data 收不到')
        return 1

    y0 = math.degrees(d.yaw())
    st = mw.stats()
    print('  起始 yaw = %.1f°' % y0)
    print('  起始地图: %s' % (st,))
    print('')
    print('  ⚠️ 机器人即将移动')
    for i in (5, 4, 3, 2, 1):
        sys.stdout.write('     %d...\r' % i)
        sys.stdout.flush()
        time.sleep(1.0)
    print('')

    t0 = time.time()
    log = []

    try:
        # ---- 1. 朝东，走到东墙 ----
        print('')
        print('  [1/4] 朝东，走到东墙')
        face(d, YAW_EAST)
        mv = walk(d, 2.0, '东向')
        log.append(('东', mv))
        st = mw.stats(); print('    地图: %s' % (st,))

        # ---- 2. 朝西，走到西墙 ----
        print('')
        print('  [2/4] 掉头朝西，走到西墙（贯穿客厅长轴）')
        face(d, YAW_WEST)
        mv = walk(d, 3.2, '西向')
        log.append(('西', mv))
        st = mw.stats(); print('    地图: %s' % (st,))

        # ---- 3. 朝北，靠近隔墙 ----
        print('')
        print('  [3/4] 朝北，靠近客厅/房间隔墙（找门洞）')
        face(d, YAW_NORTH)
        mv = walk(d, 1.6, '北向')
        log.append(('北', mv))
        st = mw.stats(); print('    地图: %s' % (st,))

        # ---- 4. 朝南回来 ----
        print('')
        print('  [4/4] 朝南，回到客厅')
        face(d, YAW_SOUTH)
        mv = walk(d, 1.8, '南向')
        log.append(('南', mv))
        st = mw.stats(); print('    地图: %s' % (st,))

    except KeyboardInterrupt:
        print('  [中断]')
    finally:
        d.stop('航线结束')

    print('')
    print('=' * 66)
    print('  航线完成，用时 %.0f s' % (time.time() - t0))
    for name, mv in log:
        print('    %s向  移动 %.2f m' % (name, mv))
    st = mw.stats()
    print('  最终地图: %s' % (st,))
    print('  结束 yaw = %.1f°' % math.degrees(d.yaw()))
    print('=' * 66)
    return 0


if __name__ == '__main__':
    sys.exit(main())
