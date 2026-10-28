# -*- coding: utf-8 -*-
"""
localize.py —— 全局重定位：用一帧激光在整张地图上穷举匹配

## 为什么需要

AMCL 是**局部**定位：它需要一个大致正确的初始位姿，然后在附近搜索。
我们给它的初始位姿是**猜的**（"机器人在场地中间"），实测是错的 ——
AMCL 收敛到了错误假设，把机器人带到了 (-0.68, -1.55) 并 ABORTED。

→ 所以先离线做一次**全局搜索**：把当前这帧激光放到地图里所有可能的
   (x, y, yaw) 上，看哪个位姿能让扫描点**正好落在墙上**。

## 方法

1. 取一帧 `/scan_filtered`（已去掉车体自遮挡，干净）
2. 对地图做**距离变换**：每个格到最近"占用格"的距离（格）
3. 对每个候选位姿 (x, y, yaw)：
     · 把激光点变换到地图坐标系
     · 查距离变换 -> 每个点离最近墙有多远
     · 得分 = 这些距离的**均值**（越小越好，说明点正好打在墙上）
     · 同时要求端点**不在占用格里**（打到墙里说明位姿不对）
4. 输出前若干名

## 用法

    python localize.py /path/to/map.pgm /path/to/map.yaml
"""
from __future__ import print_function

import math
import sys

import numpy as np
import rospy
from collections import deque
from sensor_msgs.msg import LaserScan


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    i = 2
    fields = []
    n = len(data)
    while len(fields) < 3:
        while i < n and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < n and data[i:i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        j = i
        while j < n and not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    i += 1
    w, h, _mx = fields
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape((h, w))


def read_yaml(path):
    res, ox, oy = 0.05, 0.0, 0.0
    for line in open(path):
        t = line.strip()
        if t.startswith('resolution:'):
            res = float(t.split(':')[1])
        elif t.startswith('origin:'):
            v = t.split(':', 1)[1].strip().strip('[]').split(',')
            ox, oy = float(v[0]), float(v[1])
    return res, ox, oy


def distance_transform_bfs(free_mask):
    """到最近"非 free"格的距离（格数），多源 BFS。"""
    h, w = free_mask.shape
    INF = 1e9
    d = np.where(free_mask, INF, 0.0)
    q = deque()
    for y in range(h):
        for x in range(w):
            if not free_mask[y, x]:
                continue
            edge = (y == 0 or y == h - 1 or x == 0 or x == w - 1 or
                    not free_mask[y - 1, x] or not free_mask[y + 1, x] or
                    not free_mask[y, x - 1] or not free_mask[y, x + 1])
            if edge:
                d[y, x] = 1.0
                q.append((y, x))
    while q:
        y, x = q.popleft()
        base = d[y, x] + 1.0
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free_mask[ny, nx]:
                if base < d[ny, nx]:
                    d[ny, nx] = base
                    q.append((ny, nx))
    d[np.isinf(d)] = 0.0
    return d


def main():
    pgm = sys.argv[1]
    yml = sys.argv[2]

    arr = read_pgm(pgm)
    res, ox, oy = read_yaml(yml)
    h, w = arr.shape
    occ = arr < 50
    free = arr > 250
    print('地图 %dx%d @%.3f origin(%.2f,%.2f)' % (w, h, res, ox, oy))
    print('障碍 %d 空闲 %d' % (occ.sum(), free.sum()))

    # 距离变换：到最近占用格
    dt = distance_transform_bfs(~occ)
    print('到障碍的距离变换：最大 %.0f 格' % dt.max())

    # 取一帧扫描
    rospy.init_node('localizer', anonymous=True)
    s = rospy.wait_for_message('/scan_filtered', LaserScan, timeout=15)
    pts = []
    for i, v in enumerate(s.ranges):
        if v != v or v == float('inf') or v < 0.25 or v > 6.0:
            continue
        a = s.angle_min + i * s.angle_increment
        pts.append((v * math.cos(a), v * math.sin(a)))
    # 抽稀到 ~150 点
    step = max(1, len(pts) // 150)
    pts = np.array(pts[::step], dtype=np.float64)
    print('用 %d 个激光点做匹配' % len(pts))

    # 搜索范围：默认房间 bbox；可用命令行 3..6 个参数覆盖
    #   python localize.py map.pgm map.yaml X0 X1 Y0 Y1
    X0, X1 = -2.55, 0.95
    Y0, Y1 = -1.62, 1.60
    if len(sys.argv) >= 7:
        X0, X1 = float(sys.argv[3]), float(sys.argv[4])
        Y0, Y1 = float(sys.argv[5]), float(sys.argv[6])
        print('搜索范围（命令行指定）: x %.2f~%.2f  y %.2f~%.2f'
              % (X0, X1, Y0, Y1))
    POS_STEP = 0.06
    YAW_STEP = 5

    xs = np.arange(X0, X1 + 1e-9, POS_STEP)
    ys = np.arange(Y0, Y1 + 1e-9, POS_STEP)
    print('位置网格 %d x %d = %d' % (len(xs), len(ys), len(xs) * len(ys)))

    # 世界坐标 -> 栅格索引
    def to_idx(wx, wy):
        c = ((wx - ox) / res).astype(np.int32)
        r = (h - 1 - ((wy - oy) / res)).astype(np.int32)
        np.clip(c, 0, w - 1, out=c)
        np.clip(r, 0, h - 1, out=r)
        return r, c

    results = []
    nx, ny = len(xs), len(ys)
    gx, gy = np.meshgrid(xs, ys, indexing='ij')      # (nx, ny)
    gx = gx.ravel()
    gy = gy.ravel()

    for yaw_deg in range(0, 360, YAW_STEP):
        t = math.radians(yaw_deg)
        c_, s_ = math.cos(t), math.sin(t)
        # 旋转后的相对坐标 (N,2)
        rx = pts[:, 0] * c_ - pts[:, 1] * s_
        ry = pts[:, 0] * s_ + pts[:, 1] * c_
        # 所有位置 x 所有点 -> (P, N)
        wx = gx[:, None] + rx[None, :]
        wy = gy[:, None] + ry[None, :]
        r, c = to_idx(wx, wy)
        d = dt[r, c]
        inocc = occ[r, c]
        # 得分：平均到墙距离（越小越好）+ 命中墙内的重罚
        score = d.mean(axis=1) + inocc.mean(axis=1) * 20.0
        k = int(np.argmin(score))
        results.append((float(score[k]), float(gx[k]), float(gy[k]),
                        float(yaw_deg)))

    results.sort(key=lambda t: t[0])
    print('')
    print('  最佳候选位姿（得分越小越好，单位=格）：')
    print('  %6s  %8s  %8s  %6s' % ('得分', 'x', 'y', 'yaw'))
    for sc, x, y, yw in results[:10]:
        print('  %6.2f  %8.3f  %8.3f  %6.1f' % (sc, x, y, yw))

    best = results[0]
    print('')
    print('  >>> 建议初始位姿: x=%.3f y=%.3f yaw=%.1f°  (得分 %.2f 格 = %.3f m)'
          % (best[1], best[2], best[3], best[0], best[0] * res))
    if best[0] * res > 0.25:
        print('  [!] 最佳得分仍偏高（>0.25 m），说明当前扫描和地图匹配不好，')
        print('      可能场地改动过、或机器人被放在了地图覆盖之外。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
