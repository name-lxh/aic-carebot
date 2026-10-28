# -*- coding: utf-8 -*-
"""m28_preflight.py —— 发导航目标前的自检（只读，不动车）

## 为什么要有这个

2026-10-20 第六轮 9 个航点全部失败，日志只有一句 `Failed to get a plan`，
但真因是**起点位姿落在 lethal 膨胀带里**：
AMCL 以为机器人在 (2.62,-1.43)，那里离隔断只有 3~5 cm，
0.18 m 内切圆的格代价 = 253 —— 规划器连起点都不认，任何目标都不可能规划出来。

这种错误在日志里完全看不出来。所以在**发目标之前**必须做一次自检：

    1. 起点格的代价 < lethal_cost          （起点是否可通行）
    2. 起点到目标在**同一个连通域**里      （是否被墙隔开）
    3. 起点离墙 >= robot_radius            （物理上是否站得下，本车直径 0.5 m）

三项都过才允许发目标；否则打印"该怎么修"，不发。

## 参数从哪里来

全部从参数服务器读，保证和 move_base 实际用的一致：
    /move_base/global_costmap/robot_radius
    /move_base/global_costmap/inflation_layer/inflation_radius
    /move_base/global_costmap/inflation_layer/cost_scaling_factor
    /move_base/GlobalPlanner/lethal_cost

## 用法（车上）

    source ~/Hajimi/LXH/env.sh
    python _tools/real_test/m28_preflight.py                 # 检查 + 给结论
    python _tools/real_test/m28_preflight.py --pgm /tmp/pf  # 另存代价图（P5 PGM）

退出码：0 = 可以发目标；1 = 不要发
"""
from __future__ import print_function

import math
import sys
from collections import deque

import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid

# 赛题航点（共享地图 talos_map_1016_0808 坐标系，来自 waypoints.yaml）
GOALS = [
    ('qr_room', 0.700, -2.000),   # V4：起点对面的小房间（二维码区）
    ('A_mid', 0.780, -0.960),     # V4：A 房间中间
    ('A', 1.170, -1.260),
    ('B_door', 1.880, -1.930),
    ('B', 2.550, -2.010),
    ('C_door', 1.900, -0.950),   # V3 起改到走廊侧
    ('C', 2.690, -0.920),
    ('D_door', 1.770, 0.090),
    ('D', 2.530, 0.000),
]


def get_param(name, default):
    try:
        return float(rospy.get_param(name))
    except Exception:
        return default


def load_map(topic='/map', timeout=15):
    m = rospy.wait_for_message(topic, OccupancyGrid, timeout=timeout)
    i = m.info
    data = np.array(m.data, dtype=np.int16).reshape(i.height, i.width)
    occ = data >= 65          # 占用
    unk = data < 0            # 未知
    free = (~occ) & (~unk)
    return occ, unk, free, i


def costs_for_crop(occ, unk, info, cx, cy, half=3.0, infl=0.5, inscribed=0.18,
                   scaling=25.0, lethal=200.0):
    """在机器人周围 half*2 米的窗口里，算出与 costmap_2d 一致的代价。"""
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    h, w = occ.shape
    c0 = max(0, int((cx - half - ox) / res))
    c1 = min(w, int((cx + half - ox) / res) + 1)
    # ★ OccupancyGrid 的 row 0 = 最小 y（自下而上），与 PGM 相反
    r0 = max(0, int((cy - half - oy) / res))
    r1 = min(h, int((cy + half - oy) / res) + 1)
    sub_occ = occ[r0:r1, c0:c1]
    sub_unk = unk[r0:r1, c0:c1]
    H, W = sub_occ.shape

    # 该窗口内所有障碍格的世界坐标
    rr, cc = np.nonzero(sub_occ)
    obs_x = ox + (c0 + cc + 0.5) * res
    obs_y = oy + (r0 + rr + 0.5) * res
    cell_x = ox + (c0 + np.arange(W) + 0.5) * res
    cell_y = oy + (r0 + np.arange(H) + 0.5) * res

    # 只有 infl 范围内的障碍会影响代价，所以按"障碍中心 + 邻域图章"算精确欧氏距离
    dist = np.full((H, W), 9.9, dtype=np.float32)
    R = int(math.ceil((infl + 0.15) / res))
    nd = 0
    for ox_, oy_ in zip(obs_x, obs_y):
        rr0 = max(0, int((oy_ - oy) / res) - R - r0)
        rr1 = min(H, int((oy_ - oy) / res) + R + 1 - r0)
        cc0 = max(0, int((ox_ - ox) / res) - R - c0)
        cc1 = min(W, int((ox_ - ox) / res) + R + 1 - c0)
        if rr0 >= rr1 or cc0 >= cc1:
            continue
        dd = np.sqrt((cell_x[None, cc0:cc1] - ox_) ** 2 + (cell_y[rr0:rr1, None] - oy_) ** 2)
        np.minimum(dist[rr0:rr1, cc0:cc1], dd, out=dist[rr0:rr1, cc0:cc1])
        nd += 1

    cost = np.zeros((H, W), dtype=np.float32)
    cost[sub_unk] = 255.0
    inner = dist <= inscribed
    outer = (~inner) & (dist <= infl)
    cost[inner] = 253.0
    cost[outer] = np.maximum(cost[outer],
                             252.0 * np.exp(-scaling * (dist[outer] - inscribed)))
    cost[cost > 255] = 255
    return cost, dist, sub_occ, (r0, r1, c0, c1), nd


def world_to_sub(x, y, info, box, half_shape):
    r0, r1, c0, c1 = box
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    h = info.height
    c = int((x - ox) / res) - c0
    r = int((y - oy) / res) - r0
    if 0 <= r < half_shape[0] and 0 <= c < half_shape[1]:
        return r, c
    return None


def bfs(passable, start):
    H, W = passable.shape
    seen = np.zeros((H, W), dtype=bool)
    if not passable[start]:
        return seen
    dq = deque([start])
    seen[start] = True
    while dq:
        r, c = dq.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and passable[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                dq.append((rr, cc))
    return seen


def save_pgm(path, cost, lethal):
    """代价图存成 PGM，方便拷回来看（红=致死带由本地脚本着色）。"""
    img = np.where(cost >= 254, 0, np.where(cost >= lethal, 90,
                                            np.where(cost > 0, 190, 255))).astype(np.uint8)
    img = img[::-1, :]   # 存成 PGM（自下而下 -> 自上而下），方便直接看图
    with open(path, 'wb') as f:
        f.write('P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def main():
    args = sys.argv[1:]
    pgm_out = None
    if '--pgm' in args:
        pgm_out = args[args.index('--pgm') + 1]

    rospy.init_node('nav_preflight', anonymous=True)

    radius = get_param('/move_base/global_costmap/robot_radius', 0.18)
    infl = get_param('/move_base/global_costmap/inflation_layer/inflation_radius', 0.5)
    scaling = get_param('/move_base/global_costmap/inflation_layer/cost_scaling_factor', 25.0)
    lethal = get_param('/move_base/GlobalPlanner/lethal_cost', 253.0)

    occ, unk, free, info = load_map()
    print('════════ 导航前自检 ════════')
    print('  地图 %dx%d @%.3f origin(%.2f,%.2f)'
          % (info.width, info.height, info.resolution,
             info.origin.position.x, info.origin.position.y))
    print('  参数 robot_radius=%.2f  inflation=%.2f  cost_scaling=%.0f  lethal_cost=%.0f'
          % (radius, infl, scaling, lethal))
    print('  已知空闲 %.2f m²  未知 %.2f m²'
          % (free.sum() * info.resolution ** 2, unk.sum() * info.resolution ** 2))

    if '--pose' in args:
        k = args.index('--pose')
        x, y = float(args[k + 1]), float(args[k + 2])
        yaw = float(args[k + 3]) if len(args) > k + 3 else 0.0
    else:
        x = y = yaw = None

    if x is None:
      l = tf.TransformListener()
      try:
        l.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(6.0))
        (x, y, z), (qx, qy, qz, qw) = l.lookupTransform('map', 'base_footprint', rospy.Time(0))
        yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                      1 - 2 * (qy * qy + qz * qz)))
      except Exception as e:
        print('  [FAIL] 拿不到 map->base_footprint：%r' % e)
        print('         先确认 amcl 在跑、初始位姿已设置（或加 --pose x y yaw）')
        return 1

    print('  当前位姿 (%.3f, %.3f, %.1f°)' % (x, y, yaw))
    cost, dist, sub_occ, box, n_obs = costs_for_crop(
        occ, unk, info, x, y, half=3.0, infl=infl, inscribed=radius,
        scaling=scaling, lethal=lethal)
    st = world_to_sub(x, y, info, box, cost.shape)
    if st is None:
        print('  [FAIL] 机器人不在代价图窗口内')
        return 1
    r0, c0 = st
    d_start = float(dist[r0, c0])
    c_start = float(cost[r0, c0])
    print('  起点: 离最近障碍 %.3f m  格代价 %.0f  （窗口内障碍 %d 格）'
          % (d_start, c_start, n_obs))

    passable = cost < lethal
    seen = bfs(passable, st)
    print('  起点可通行: %s   起点连通域 %d 格'
          % ('YES' if passable[r0, c0] else 'NO', int(seen.sum())))

    ok = True
    if c_start >= lethal:
        ok = False
        print('  ✗ 起点格代价 %.0f >= lethal_cost %.0f —— 规划器会直接拒绝，'
              '这就是 "Failed to get a plan" 刷屏的原因' % (c_start, lethal))
    if d_start < radius:
        ok = False
        print('  ✗ 起点离墙 %.3f m < robot_radius %.2f m —— 物理上站不下'
              '（本车直径 0.5 m，半径 0.25 m）' % (d_start, radius))

    print('\n  目标点检查:')
    print('    %-8s %6s %6s %6s %s' % ('目标', 'cost', '离墙m', '可达', ''))
    for name, gx, gy in GOALS:
        gs = world_to_sub(gx, gy, info, box, cost.shape)
        if gs is None:
            print('    %-8s  (%.2f,%.2f) 不在窗口内' % (name, gx, gy))
            continue
        gr, gc = gs
        gc_cost = float(cost[gr, gc])
        gc_d = float(dist[gr, gc])
        reach = bool(seen[gr, gc]) if passable[gr, gc] else False
        flag = ''
        if gc_cost >= lethal:
            flag = '✗ 目标在致死带'
        elif not reach:
            flag = '✗ 与起点不连通'
        print('    %-8s %6.0f %6.2f %6s %s'
              % (name, gc_cost, gc_d, 'YES' if reach else 'NO', flag))

    if pgm_out:
        save_pgm(pgm_out, cost, lethal)
        print('\n  代价图已存 %s（P5 PGM，窗口 %.1f×%.1f m）'
              % (pgm_out, cost.shape[1] * info.resolution, cost.shape[0] * info.resolution))

    print('')
    if ok:
        print('  ✅ 自检通过，可以发目标')
    else:
        print('  ⛔ 不要发目标。修法：')
        print('     · 把车挪到房间/场地中间（离墙 >= %.2f m 以上），' % max(0.4, radius + 0.15))
        print('     · 重新定位（localize / rviz 2D Pose Estimate），要求匹配得分 < 0.10 m')
        print('     · 再跑一次本自检，通过后才发航点')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
