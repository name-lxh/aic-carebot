# -*- coding: utf-8 -*-
"""
sim_cost_sim.py — 离线复现 costmap_2d 的膨胀计算

## 为什么离线算

在 VM 上"改参数 → 起仿真 → 跑测试"一轮好几分钟，
而且只能看到最终结果，看不出"到底哪条路被堵了"。

离线复现的好处：
  · 秒级出结果，可以快速扫参数
  · 能**打印整张成本图**，直接看到通不通
  · 能算出"有多少面积落到致命/高代价"

## 复现的是哪部分

只管**膨胀层**（inflation_layer），因为问题就在它：

    costmap_2d 的膨胀代价公式（与源码一致）：

        d = 到最近障碍的距离
        if d <= inscribed_radius:      cost = 254（内切圆内 = 致命）
        elif d <= inflation_radius:
            factor = (inflation_radius - d) / (inflation_radius - inscribed)
            cost   = (253 - 1) * exp(-cost_scaling_factor * factor) + 1
        else:                          cost = 0

    其中 inscribed_radius = robot_radius = 0.18（圆形 footprint 时）
          inflation_radius = 0.22

    ⚠️ `cost_scaling_factor` 越大，代价衰减越快 —— 但**致命区
       （d <= inscribed_radius）不受它影响**，只由 robot_radius 决定。
       这一点很容易搞混：调 cost_scaling_factor 不会让"内切圆内的格"
       变得可通行。

用法：
    python sim_cost_sim.py                      # 用当前配置分析
    python sim_cost_sim.py --inflate 0.20       # 试不同的膨胀半径
    python sim_cost_sim.py --robot 0.16         # 试不同的机器人半径
    python sim_cost_sim.py --sweep              # 扫一组参数对比
"""

from __future__ import print_function

import argparse
import io
import math
import os
import sys

# ---------------------------------------------------------------------------
# 输出编码：用共享实现（见 sim_arena.py 里的详细说明）
# ---------------------------------------------------------------------------
# ⚠️ 原来这里内联了一段自己写的 `codecs.getwriter` 包装。
#    能用，但问题是**每个脚本各写一份**，本项目已经有 5 处重复，
#    其中还出现过"对已包装的流再包一层导致 UnicodeDecodeError"。
#    统一走 `utf8_stdio`，只留一份实现。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
try:
    from utf8_stdio import setup_all as setup_stdout
except ImportError:
    # ⚠️ 退化实现**也必须容忍字节串**（详见 auto_points.py 的说明）：
    #    裸 `codecs.getwriter` 在 Py2 下会把 traceback 的**字节串**
    #    先 decode('ascii') 而崩掉，真正要看的异常反而看不到。
    def setup_stdout():
        import codecs

        class _Utf8Writer(codecs.StreamWriter):
            def write(self, s):
                if s is None:
                    return
                try:
                    if isinstance(s, bytes):
                        self.stream.write(s)
                        return
                    self.stream.write(s.encode('utf-8',
                                                'backslashreplace'))
                except Exception:
                    try:
                        self.stream.write(repr(s))
                    except Exception:
                        pass

        for _name in ('stdout', 'stderr'):
            _s = getattr(sys, _name, None)
            if _s is None:
                continue
            try:
                _buf = getattr(_s, 'buffer', _s)
                setattr(sys, _name,
                        _Utf8Writer(_buf, errors='backslashreplace'))
            except Exception:
                pass

setup_stdout()

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, '..', 'maps')
DEFAULT_PGM = os.path.join(MAPS, 'sim_arena.pgm')


def read_pgm(path):
    with open(path, 'rb') as f:
        b = f.read()
    i = 0
    tok = []
    while len(tok) < 4:
        while b[i:i + 1].isspace():
            i += 1
        if b[i:i + 1] == b'#':
            while b[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not b[j:j + 1].isspace():
            j += 1
        tok.append(b[i:j])
        i = j
    i += 1
    w, h = int(tok[1]), int(tok[2])
    return w, h, b[i:i + w * h]


def build(grid, w, h):
    """把 PGM 数据转成占据布尔表。

    ROS 约定：0=占据，254=空闲。occupied_thresh/free_thresh 是
    给灰度图用的，二值 PGM 直接按 128 分界。
    """
    occ = [[grid[y * w + x] < 128 for x in range(w)] for y in range(h)]
    return occ


def dist_to_obstacle(occ, w, h, res):
    """每个格到最近障碍（含地图外）的距离。BFS 两遍 chamfer 近似。"""
    INF = 1e9
    d = [[0.0 if occ[y][x] else INF for x in range(w)] for y in range(h)]
    # 地图外视为障碍
    for y in range(h):
        for x in range(w):
            if d[y][x] == 0.0:
                continue
            edge = min(x, y, w - 1 - x, h - 1 - y) * res
            if edge < d[y][x]:
                d[y][x] = edge
    for y in range(h):
        for x in range(w):
            if d[y][x] == 0.0:
                continue
            best = d[y][x]
            for dy, dx, k in ((-1, 0, 1.0), (0, -1, 1.0),
                              (-1, -1, 1.4142), (-1, 1, 1.4142)):
                yy, xx = y + dy, x + dx
                if 0 <= yy < h and 0 <= xx < w:
                    v = d[yy][xx] + k * res
                    if v < best:
                        best = v
            d[y][x] = best
    for y in range(h - 1, -1, -1):
        for x in range(w - 1, -1, -1):
            if d[y][x] == 0.0:
                continue
            best = d[y][x]
            for dy, dx, k in ((1, 0, 1.0), (0, 1, 1.0),
                              (1, 1, 1.4142), (1, -1, 1.4142)):
                yy, xx = y + dy, x + dx
                if 0 <= yy < h and 0 <= xx < w:
                    v = d[yy][xx] + k * res
                    if v < best:
                        best = v
            d[y][x] = best
    return d


def inflate(occ, dist, w, h, robot_r, infl_r, scaling):
    """按 costmap_2d 的公式算代价。

    返回 cost 表：0 = 空闲，1..252 = 梯度，253 = 内切圆内(致命)，
    -1 = 障碍本身。
    """
    cost = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            if occ[y][x]:
                cost[y][x] = -1          # 障碍
                continue
            d = dist[y][x]
            if d <= robot_r:
                cost[y][x] = 253         # 内切圆内 → 致命
            elif d <= infl_r:
                if infl_r <= robot_r:
                    cost[y][x] = 1
                else:
                    f = (infl_r - d) / (infl_r - robot_r)
                    cost[y][x] = int(252 * math.exp(-scaling * f)) + 1
            else:
                cost[y][x] = 0
    return cost


def stats(cost, w, h, free_cost_max=100):
    """统计构成。free_cost_max = 规划器认为"可通行"的代价上限。

    ⚠️ GlobalPlanner 的 lethal_cost 默认 253 —— 也就是
       代价 >= 253 才算不可通行。所以"梯度的 252"理论上是能走的，
       只是路径代价高。但 DWA 的 occdist_scale 会让它绕开。
    """
    n_obs = sum(1 for y in range(h) for x in range(w) if cost[y][x] == -1)
    n_lethal = sum(1 for y in range(h) for x in range(w)
                   if cost[y][x] == 253)
    n_grad = sum(1 for y in range(h) for x in range(w)
                 if 0 < cost[y][x] < 253)
    n_free = sum(1 for y in range(h) for x in range(w)
                 if cost[y][x] == 0)
    tot = float(w * h)
    return {
        'obs': n_obs, 'lethal': n_lethal,
        'grad': n_grad, 'free': n_free,
        'pct_obs': 100 * n_obs / tot,
        'pct_lethal': 100 * n_lethal / tot,
        'pct_grad': 100 * n_grad / tot,
        'pct_free': 100 * n_free / tot,
    }


def show(cost, w, h, ox, oy, res, pts, title):
    """打印成本图的字符画。

    '#' 障碍   'X' 致命   '1'-'9' 代价梯度（越大越差）   '.' 空闲
    点位用 B/C/D/Q/A/S 标出来。
    """
    marks = {}
    for ch, name in (('S', 'start'), ('Q', 'qr_area'),
                     ('A', 'point_A'), ('B', 'room_B'),
                     ('C', 'room_C'), ('D', 'room_D')):
        if name not in pts:
            continue
        x, y = pts[name]
        gx = int((x - ox) / res)
        gy = h - 1 - int((y - oy) / res)
        marks[(gy, gx)] = ch
    print(u'  %s' % title)
    for y in range(h):
        row = []
        for x in range(w):
            if (y, x) in marks:
                row.append(marks[(y, x)])
                continue
            c = cost[y][x]
            if c == -1:
                row.append('#')
            elif c >= 253:
                row.append('X')
            elif c == 0:
                row.append('.')
            else:
                row.append(str(min(9, max(1, c // 28 + 1))))
        print(u'    %s' % u''.join(row))


def connectivity(cost, w, h, pts, ox, oy, res, lethal=253):
    """检查点位之间是否连通（4 邻域洪水填充）。"""
    passable = [[cost[y][x] < lethal for x in range(w)] for y in range(h)]

    def flood(sy, sx):
        if not passable[sy][sx]:
            return None
        seen = [[False] * w for _ in range(h)]
        stack = [(sy, sx)]
        seen[sy][sx] = True
        n = 0
        while stack:
            y, x = stack.pop()
            n += 1
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                yy, xx = y + dy, x + dx
                if 0 <= yy < h and 0 <= xx < w \
                        and not seen[yy][xx] and passable[yy][xx]:
                    seen[yy][xx] = True
                    stack.append((yy, xx))
        return seen

    def cell(name):
        x, y = pts[name]
        return (h - 1 - int((y - oy) / res), int((x - ox) / res))

    if 'start' not in pts:
        return {}
    sy, sx = cell('start')
    if not passable[sy][sx]:
        return {'_start': '★ 起点本身不可通行'}
    seen = flood(sy, sx)
    out = {}
    for name in pts:
        gy, gx = cell(name)
        out[name] = bool(seen[gy][gx]) if (0 <= gy < h and 0 <= gx < w) \
            else False
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pgm', default=DEFAULT_PGM)
    ap.add_argument('--robot', type=float, default=0.18)
    ap.add_argument('--inflate', type=float, default=0.22)
    ap.add_argument('--scaling', type=float, default=10.0)
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--sweep', action='store_true')
    args = ap.parse_args()

    sys.path.insert(0, os.path.abspath(os.path.join(HERE)))
    import sim_arena as S

    w, h, grid = read_pgm(args.pgm)
    res = S.RESOLUTION
    ox = -w * res / 2.0
    oy = -h * res / 2.0
    occ = build(grid, w, h)
    dist = dist_to_obstacle(occ, w, h, res)
    pts = {k: (v[0], v[1]) for k, v in S.POINTS.items()}

    print(u'')
    print(u'  地图 %d×%d @%.2f m  origin (%.2f, %.2f)'
          % (w, h, res, ox, oy))
    print(u'  机器人半径 %.2f   膨胀半径 %.2f   代价衰减 %.1f'
          % (args.robot, args.inflate, args.scaling))

    if args.sweep:
        print(u'')
        print(u'  %-8s %-8s %-8s %-9s %-9s %-9s %s'
              % (u'机器人', u'膨胀', u'衰减', u'空闲%', u'梯度%',
                 u'致命%', u'点位可达'))
        print(u'  ' + u'-' * 78)
        for robot in (0.14, 0.16, 0.18):
            for infl in (0.16, 0.20, 0.22, 0.28):
                for sc in (5.0, 10.0):
                    if infl < robot:
                        continue
                    c = inflate(occ, dist, w, h, robot, infl, sc)
                    st = stats(c, w, h)
                    conn = connectivity(c, w, h, pts, ox, oy, res)
                    nconn = sum(1 for k, v in conn.items() if v)
                    tot = len(conn)
                    bad = [k for k, v in conn.items() if not v]
                    print(u'  %-8.2f %-8.2f %-8.1f %-9.0f %-9.0f %-9.0f '
                          u'%d/%d %s'
                          % (robot, infl, sc, st['pct_free'],
                             st['pct_grad'], st['pct_lethal'],
                             nconn, tot,
                             (u'  缺: ' + u','.join(bad)) if bad else u''))
        print(u'')
        return 0

    cost = inflate(occ, dist, w, h, args.robot, args.inflate, args.scaling)
    st = stats(cost, w, h)
    print(u'')
    print(u'  构成：空闲 %.0f%%   代价梯度 %.0f%%   致命 %.0f%%   障碍 %.0f%%'
          % (st['pct_free'], st['pct_grad'], st['pct_lethal'],
             st['pct_obs']))

    print(u'')
    print(u'  各点位：')
    for name in ('start', 'qr_area', 'point_A',
                 'room_B', 'room_C', 'room_D'):
        if name not in pts:
            continue
        x, y = pts[name]
        gx = int((x - ox) / res)
        gy = h - 1 - int((y - oy) / res)
        c = cost[gy][gx]
        d = dist[gy][gx]
        lbl = (u'障碍' if c == -1 else
               u'★ 致命(内切圆内)' if c >= 253 else
               u'★ 代价 %d' % c if c > 0 else u'空闲')
        print(u'    %-14s (%.2f,%.2f)  离障 %.2f m  %s'
              % (name, x, y, d, lbl))

    conn = connectivity(cost, w, h, pts, ox, oy, res)
    print(u'')
    print(u'  连通性（从起点洪水填充，看能不能走到）：')
    for name, ok in sorted(conn.items()):
        if name.startswith('_'):
            print(u'    %s' % ok)
            continue
        print(u'    %-14s %s' % (name, u'✔ 可达' if ok else u'✘ 不可达'))

    if args.show:
        print(u'')
        show(cost, w, h, ox, oy, res, pts, u'成本图：')
    print(u'')
    return 0


if __name__ == '__main__':
    sys.exit(main())
