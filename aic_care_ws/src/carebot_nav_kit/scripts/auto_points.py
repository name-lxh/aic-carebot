# -*- coding: utf-8 -*-
"""
auto_points.py — 从场地几何自动算出最优点位

## 为什么不让程序自己挑

手工试点位试了好几轮都没试对，原因是**靠眼睛看 ASCII 图估距离很不准**：

    · 墙在哪一列、点位在第几格，人眼数格容易错
    · "离墙多远"要算欧氏距离，不是看左右各几格
    · 场地改一次布局，手工点位就全废

而这件事**机器算起来很简单**：
    1. 算出每个可通行格到最近墙的距离（距离变换）
    2. 在指定区域内挑"距离最大"的格作为点位

## 分区

把场地按区域划分，每区挑离墙最远的点：

    room_B  左上区    room_C  中上区    room_D  右上区
    qr_area 客厅左    point_A 客厅右     start   底部开口

用法（输出可直接粘进 sim_arena.py 的 POINTS）：
    python auto_points.py
"""

import os
import sys

# ---------------------------------------------------------------------------
# 输出编码：用共享实现（详见 sim_arena.py 里的说明）
# ---------------------------------------------------------------------------
# ⚠️ 这里原来内联了一段 `reconfigure` 优先 + `codecs.getwriter` 兜底的代码。
#    逻辑本身是对的（知道 reconfigure 是 3.7+ 才有），
#    但问题是**每个脚本各写一份**，本项目已有 5 处重复实现，
#    其中出现过"对已包装的流再包一层 → UnicodeDecodeError"这种事故。
#
#    统一走 `utf8_stdio.setup_all()`：唯一一份、且容忍字节串输入。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
try:
    from utf8_stdio import setup_all as setup_stdout
except ImportError:
    # ⚠️ 退化实现**也必须容忍字节串**。
    #
    # 原来这里用的是裸 `codecs.getwriter('utf-8')` —— 看着是标准做法，
    # 但 Py2 下不够：标准 StreamWriter 只接受 unicode，而
    # `traceback.print_exc()` / `logging.StreamHandler` 会往流里写
    # **字节串**，于是先 `decode('ascii')` → 崩：
    #     UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5
    # 后果是"为了看清异常而打印 traceback 时反而崩掉"。
    #
    # 正确做法：自定义 StreamWriter 子类覆写 write()。
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

sys.path.insert(0, '.')
import sim_arena as S


def build_dist(grid, res):
    """距离变换：每个可通行格到最近墙（或地图边界）的距离。

    用简单的多源 BFS 近似（按格算欧氏距离），
    场地只有 53×53，性能完全够。
    """
    h = len(grid)
    w = len(grid[0])
    INF = 1e9
    dist = [[INF] * w for _ in range(h)]
    # 墙体格距离 0
    for y in range(h):
        for x in range(w):
            if grid[y][x] == '#':
                dist[y][x] = 0.0
    # 边界外也是墙 → 把边界格的"到边界距离"也算进去
    for y in range(h):
        for x in range(w):
            if dist[y][x] == 0.0:
                continue
            d = min(x, y, w - 1 - x, h - 1 - y) * res
            if d < dist[y][x]:
                dist[y][x] = d
    # 两遍扫描做欧氏近似（chamfer）
    for y in range(h):
        for x in range(w):
            if dist[y][x] == 0.0:
                continue
            best = dist[y][x]
            for dy, dx, k in ((-1, 0, 1.0), (0, -1, 1.0),
                              (-1, -1, 1.4142), (-1, 1, 1.4142)):
                yy, xx = y + dy, x + dx
                if 0 <= yy < h and 0 <= xx < w:
                    best = min(best, dist[yy][xx] + k * res)
            dist[y][x] = best
    for y in range(h - 1, -1, -1):
        for x in range(w - 1, -1, -1):
            if dist[y][x] == 0.0:
                continue
            best = dist[y][x]
            for dy, dx, k in ((1, 0, 1.0), (0, 1, 1.0),
                              (1, 1, 1.4142), (1, -1, 1.4142)):
                yy, xx = y + dy, x + dx
                if 0 <= yy < h and 0 <= xx < w:
                    best = min(best, dist[yy][xx] + k * res)
            dist[y][x] = best
    return dist


def best_in(dist, grid, res, ox, oy, x0, x1, y0, y1):
    """在指定世界坐标矩形里找离墙最远的点。"""
    h = len(grid)
    w = len(grid[0])
    bx = by = None
    bd = -1.0
    for y in range(h):
        for x in range(w):
            if grid[y][x] == '#':
                continue
            wx = ox + (x + 0.5) * res
            wy = oy + (h - 1 - y + 0.5) * res
            if not (x0 <= wx <= x1 and y0 <= wy <= y1):
                continue
            if dist[y][x] > bd:
                bd = dist[y][x]
                bx, by = wx, wy
    return bx, by, bd


def main():
    grid = S.build_grid()
    h = len(grid)
    w = len(grid[0])
    res = S.RESOLUTION
    ox = -w * res / 2.0
    oy = -h * res / 2.0

    print(u'')
    print(u'  场地 %d×%d 格 @%.2fm = %.1f × %.1f m'
          % (w, h, res, w * res, h * res))
    print(u'  世界坐标 x ∈ [%.2f, %.2f]  y ∈ [%.2f, %.2f]'
          % (ox, ox + w * res, oy, oy + h * res))

    dist = build_dist(grid, res)

    # 各区域的搜索矩形（世界坐标）
    X1 = ox + w * res
    Y1 = oy + h * res
    regions = [
        ('start',   ox, X1,      oy, oy + h * res * 0.20),   # 底部开口
        ('qr_area', ox, ox + w * res * 0.45,
                    oy + h * res * 0.30, oy + h * res * 0.55),
        ('point_A', ox + w * res * 0.45, ox + w * res * 0.70,
                    oy + h * res * 0.30, oy + h * res * 0.55),
        ('room_B',  ox, ox + w * res * 0.30,
                    oy + h * res * 0.62, Y1),
        ('room_C',  ox + w * res * 0.30, ox + w * res * 0.62,
                    oy + h * res * 0.62, Y1),
        ('room_D',  ox + w * res * 0.62, X1,
                    oy + h * res * 0.62, Y1),
    ]

    print(u'')
    print(u'  ★ 自动挑出的点位（间隙 = 到最近墙的距离）：')
    out = {}
    for name, x0, x1, y0, y1 in regions:
        bx, by, bd = best_in(dist, grid, res, ox, oy, x0, x1, y0, y1)
        if bx is None:
            print(u'    %-10s  该区域没有可通行格！' % name)
            continue
        out[name] = (round(bx, 2), round(by, 2))
        flag = u'OK' if bd >= 0.45 else u'★ 太近'
        print(u'    %-10s (%5.2f, %5.2f)  间隙 %.2f m  %s'
              % (name, bx, by, bd, flag))

    print(u'')
    print(u'  --- 可直接粘进 sim_arena.py ---')
    print(u'  POINTS = {')
    yaw = {'start': 1.5708}
    for name in ('start', 'qr_area', 'point_A',
                 'room_B', 'room_C', 'room_D'):
        if name not in out:
            continue
        x, y = out[name]
        print(u"      '%s':%s(%5.2f, %5.2f, %.4f),"
              % (name, ' ' * (9 - len(name)), x, y, yaw.get(name, 0.0)))
    print(u'  }')
    print(u'')

    # 顺便报告最窄通道（能不能过）
    need = 0.18 + 0.22
    print(u'  最窄通道检查（机器人需要 %.2f m 间隙）：' % need)
    inner = [(x, y) for y in range(1, h - 1) for x in range(1, w - 1)
             if grid[y][x] != '#']
    if inner:
        mn = min(dist[y][x] for x, y in inner)
        tight = [(x, y) for x, y in inner if dist[y][x] < need]
        print(u'    所有可通行格中最小间隙 %.2f m' % mn)
        if tight:
            print(u'    ★ 有 %d 个格间隙不足 %.2f m —— 机器人过不去，'
                  u'那里会被膨胀堵死' % (len(tight), need))
            print(u'      （这些格不影响点位，但会让规划器绕路）')
        else:
            print(u'    OK 全部可通行格的间隙都够')
    print(u'')


if __name__ == '__main__':
    main()
