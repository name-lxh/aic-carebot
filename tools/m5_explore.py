# -*- coding: utf-8 -*-
"""
m5_explore.py —— 前沿(frontier)驱动的自由探索建图

## 为什么不用之前那个 explore_map.py

`explore_map.py` 是**反应式**的：只看"前方够不够空"，不空就转 45°。
在 1.2 m 深的客厅里它会在原地来回摆：

    [54s] 前方仅 0.42 m(左 0.82 右 1.90) → 转 -45°
    [57s] 前方仅 0.50 m(左 1.47 右 0.93) → 转 +45°
    [61s] 前方仅 0.38 m(左 0.82 右 1.61) → 转 -45°

因为它**不知道哪里还没扫过**，只是机械地躲障碍。

## 本脚本的做法：朝"未知区域最多"的方向走

每一轮：

  1. 从 TF 拿机器人在**地图坐标系**里的位置和朝向
  2. 把周围 360° 分成 12 个方位（30° 一格）
  3. 对每个方位，沿射线统计 **0.6~2.5 m 区间里的"未知"格子数**
     —— 未知越多，说明那个方向越没扫过
  4. **用当前这一帧激光**判断每个方位的净空
     （激光是 360° 的，所以不用转身就能知道各方向的净空）
  5. 在"净空 >= 0.75 m"的方位里，挑**未知最多**的那个
  6. 转过去 → 小步前进
  7. 如果所有方向都没有"又空又未知"的：
       · 背后够空 → **后退**一步（用户要求可以前后走）
       · 否则 → 转向净空最大的方向（纯转，靠边走）

## 底盘能力限制（实测）

`base_control/script/stm32.py` 的 `smooth_cmd_sender` 里：

    outputdata[4..5] = current_x     # 前进/后退
    outputdata[6..7] = 0x00          # ★ linear.y **被硬编码为 0**
    outputdata[8..9] = current_rot_z # 旋转

所以**不能横移**，只能前进/后退/旋转。"左右"靠转身实现。

## 安全

  · MIN_VALID=0.35（车体自遮挡以下忽略）
  · FRONT_STOP=0.50（前方 0.5 m 急停）
  · 看门狗兜底（底盘没有 cmd_vel 超时归零）
  · 每步硬超时；总时长由 --minutes 控制
"""
from __future__ import print_function

import math
import sys
import time

import rospy
import tf
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')
from realbot import RealBot, FRONT_STOP, MIN_VALID   # noqa: E402

# ---- 参数 ----------------------------------------------------------------
N_BINS = 12                # 方位分几格（30°）
RAY_MIN = 0.60             # m  射线统计起点（跳过车体附近）
RAY_MAX = 2.50             # m  射线统计终点
CLEAR_OK = 0.90            # m  这个方位的净空至少这么大才敢过去
                           # ⚠️ 护栏已改用滤波后的 /scan_filtered（min ~0.596），
                           #    FRONT_STOP=0.60，所以这里的门槛必须**高于** 0.60
STEP = 0.12                # m  每步前进/后退（小步更安全）
V = 0.10                   # m/s
MIN_GAIN = 12              # 至少这么多未知格才值得跑一趟（否则跳过）
REVISIT_R = 0.45           # m  离已访问点这么近就算重复

# 相邻方位如果净空未知，用这个兜底
DEFAULT_CLEAR = 9.9


def norm180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def norm360(a):
    a = math.fmod(a, 360.0)
    if a < 0:
        a += 360.0
    return a


class MapGrid(object):
    def __init__(self):
        self.map = None
        rospy.Subscriber('/map', OccupancyGrid, self._cb, queue_size=1)

    def _cb(self, m):
        self.map = m

    def ready(self):
        return self.map is not None

    def unknown_count(self, wx, wy, bearing_deg):
        """从 (wx,wy) 沿 bearing 射线，统计 RAY_MIN~RAY_MAX 里的未知格数。"""
        if self.map is None:
            return 0
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        w = self.map.info.width
        h = self.map.info.height
        d = self.map.data

        br = math.radians(bearing_deg)
        cb = math.cos(br)
        sb = math.sin(br)
        cnt = 0
        r = RAY_MIN
        while r <= RAY_MAX:
            # 射线加一点横向抖动，避免正交方向漏采样
            for off in (-0.10, 0.0, 0.10):
                x = wx + r * cb - off * sb
                y = wy + r * sb + off * cb
                c = int((x - ox) / res)
                rr = int((y - oy) / res)
                if 0 <= c < w and 0 <= rr < h:
                    if d[rr * w + c] < 0:      # -1 = 未知
                        cnt += 1
            r += res * 2.0
        return cnt

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

    rospy.init_node('m5_explore', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv), verbose=False)
    mg = MapGrid()
    lst = tf.TransformListener()

    print('')
    print('=' * 66)
    print('  前沿驱动自由探索（预算 %.1f 分钟）' % minutes)
    print('=' * 66)
    print('  MIN_VALID=%.2f  FRONT_STOP=%.2f  净空门槛=%.2f'
          % (MIN_VALID, FRONT_STOP, CLEAR_OK))

    if not d.ready(timeout=20):
        print('  [FAIL] /scan 或 /imu_data 收不到')
        return 1

    # 等地图有数据
    t0 = time.time()
    while not mg.ready() and time.time() - t0 < 20:
        time.sleep(0.5)
    if not mg.ready():
        print('  [FAIL] /map 收不到')
        return 1

    print('  地图: %s' % (mg.stats(),))
    print('')
    print('  ⚠️ 机器人即将自由探索')
    for i in (5, 4, 3, 2, 1):
        sys.stdout.write('     %d...\r' % i)
        sys.stdout.flush()
        time.sleep(1.0)
    print('')

    t_end = time.time() + minutes * 60.0
    visited = []
    n_fwd = n_back = n_turn = 0
    n_skip = 0
    seen_map = set()

    def pose_map():
        """返回 (map 系下的 x, y, yaw_deg)，失败返回 None。"""
        try:
            lst.waitForTransform('/map', '/base_footprint',
                                 rospy.Time(0), rospy.Duration(1.0))
            (tr, rot) = lst.lookupTransform('/map', '/base_footprint',
                                            rospy.Time(0))
            q = rot
            yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                             1 - 2 * (q[1] * q[1] + q[2] * q[2]))
            return (tr[0], tr[1], math.degrees(yaw))
        except Exception:
            return None

    try:
        while time.time() < t_end and not rospy.is_shutdown():
            el = time.time() - t0
            pm = pose_map()
            if pm is None:
                print('  [%5.0fs] 拿不到 map->base_footprint，等' % el)
                time.sleep(1.0)
                continue
            mx, my, myaw = pm
            imu_yaw = math.degrees(d.yaw())
            # map 系与 IMU 系的朝向偏置
            off = norm180(myaw - imu_yaw)

            # ---- 1. 给每个方位打分：未知多少 + 净空多少 ----
            cands = []
            for k in range(N_BINS):
                bear = k * (360.0 / N_BINS)          # map 系方位
                rel = norm180(bear - myaw)           # 相对车头
                clear = d.sector_min_robust(math.radians(rel),
                                            half_deg=22.0)
                if clear is None:
                    continue
                unk = mg.unknown_count(mx, my, bear)
                # 已访问附近的方向打点折，减少来回兜圈
                pen = 0.0
                for (vx, vy, _va) in visited[-8:]:
                    if math.hypot(mx + 0.5 * math.cos(math.radians(bear)) - vx,
                                  my + 0.5 * math.sin(math.radians(bear)) - vy) \
                            < REVISIT_R:
                        pen += 0.5
                cands.append((bear, rel, clear, unk, unk * (1.0 - min(pen, 0.9))))

            # ⚠️ 自适应分级门槛 —— 实测必须这样。
            #
            #  一开始只用固定的 (净空>=0.75, 未知>=12)，结果在这台车上
            #  常常是 "候选 0/11"：客厅只有 1.2 m 深，扣掉 0.35 m 车体自遮挡，
            #  可用净空只剩 ~0.85 m，0.75 的门槛**几乎永远达不到**。
            #  于是它一直卡在"只转不走"，地图几乎不再增长（21.0% -> 21.2%）。
            #
            #  改成三级放宽：优先"又空又未知"，不行就降要求，最后只要能动就动。
            LEVELS = [
                (CLEAR_OK, MIN_GAIN),      # 0.90 / 12
                (0.82,     8),             # 退一步
                (0.76,     4),             # 再退一步（仍高于 FRONT_STOP+0.05）
            ]
            good = []
            for need_clear, need_unk in LEVELS:
                good = [c for c in cands
                        if c[2] >= need_clear and c[4] >= need_unk]
                if good:
                    good.sort(key=lambda c: -c[4])
                    break

            print('  [%5.0fs] pos(%.2f,%.2f) yaw_map=%.0f°  候选 %d/%d'
                  % (el, mx, my, myaw, len(good), len(cands)))

            if good:
                bear, rel, clear, unk, score = good[0]
                print('           选方位 %.0f°（相对 %+.0f°）净空 %.2f 未知 %d'
                      % (bear, rel, clear, unk))
                # 转到该方位（用 IMU 绝对角：目标 = 该方位 - 偏置）
                target_imu = norm180(bear - off)
                cur_imu = math.degrees(d.yaw())
                delta = norm180(target_imu - cur_imu)
                d.rotate(delta, timeout=30.0)
                n_turn += 1

                # 前进步数：净空越大走得越多，但不超过 STEP*3
                nstep = 1 if clear < 1.1 else 3
                for s in range(nstep):
                    if time.time() > t_end:
                        break
                    fm = d.sector_min_robust(0.0)
                    if fm is None or fm < FRONT_STOP + 0.10:
                        break
                    d.forward(STEP, speed=V)
                    n_fwd += 1
                visited.append((mx, my, myaw))
                continue

            # ---- 2. 没有"又空又未知"的：试着后退 ----
            back = d.sector_min_robust(math.pi, half_deg=30.0)
            if back is not None and back >= 0.72:
                print('           无合适前向目标，后退一步（后方 %.2f m）' % back)
                d.forward(-STEP, speed=V * 0.8)
                n_back += 1
                visited.append((mx, my, myaw))
                continue

            # ---- 3. 纯转找方向 ----
            best = max(cands, key=lambda c: c[2]) if cands else None
            if best is None:
                print('           没有任何有效方向，停 2 秒')
                time.sleep(2.0)
                continue
            bear, rel, clear, unk, score = best
            print('           只转不走：净空最大的方位 %.0f°（%.2f m）'
                  % (bear, clear))
            target_imu = norm180(bear - off)
            cur_imu = math.degrees(d.yaw())
            delta = norm180(target_imu - cur_imu)
            if abs(delta) > 20.0:
                d.rotate(delta, timeout=30.0)
                n_turn += 1
            else:
                # 已经朝着最空的方向了还是走不了 —— 退半步试试，再不然等一下
                back2 = d.sector_min_robust(math.pi, half_deg=35.0)
                if back2 is not None and back2 >= 0.60:
                    print('           改后退半步（后方 %.2f m）' % back2)
                    d.forward(-STEP * 0.7, speed=V * 0.8)
                    n_back += 1
                else:
                    time.sleep(2.5)
            n_skip += 1

    except KeyboardInterrupt:
        print('  [中断]')
    finally:
        d.stop('探索结束')

    print('')
    print('=' * 66)
    print('  探索结束，用时 %.0f s' % (time.time() - t0))
    print('    前进 %d 次   后退 %d 次   转向 %d 次   空转 %d 次'
          % (n_fwd, n_back, n_turn, n_skip))
    print('    最终地图: %s' % (mg.stats(),))
    pm = pose_map()
    if pm:
        print('    结束位姿: (%.2f, %.2f) yaw=%.0f°' % pm)
    print('=' * 66)
    return 0


if __name__ == '__main__':
    sys.exit(main())
