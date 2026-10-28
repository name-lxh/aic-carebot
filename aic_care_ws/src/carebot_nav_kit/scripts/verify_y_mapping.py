#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
verify_y_mapping.py — 校验"世界坐标 <-> 场地网格"的 Y 轴映射

## 为什么单独写这个

踩过一个极难查的坑：`fake_sensors.is_occupied` 少了 Y 轴翻转，
把起点世界坐标 (0, -1.90) 算成 gy=7（北边墙里），
而真正的起点走廊在 gy=46。**机器人被"放进墙里"**，
射线立刻打到 0.2~0.5 m 的"墙"，生成一圈假障碍 → 导航全废。

而且它**极难发现**：
  · 日志只说 aborted，不提坐标系
  · 洪水填充查可达性：6 个点位**全可达**（因为代价图在错误但自洽的格子上算的）
  · 单看地图 PGM 完全正常，sim_precheck 也全过
  · 激光自扫阈值从 0.22 提到 0.38 都没用（那批点不是自扫，是"墙"）

所以需要一个**专门的、把两侧打印出来对照**的校验。

本脚本用**三种方式**交叉验证同一个点：
    1. `sim_arena` 的建图顺序（ARENA[0] 是北还是南）
    2. `fake_sensors.FakeWorld.is_occupied`（仿真真正用的）
    3. 直接查 grid（绕开被测代码）

⚠️ 输出**故意全用 ASCII** —— Py2 下 stdout 包装不可靠，
   中文输出会在 `codecs.py` 里抛 UnicodeDecodeError。
   （见 `ros_logfix.setup_stdout`，那是踩过之后写对的版本。）

用法（在 VM 上）：
    cd ~/aic_care_ws
    python src/carebot_nav_kit/scripts/verify_y_mapping.py
"""

from __future__ import print_function

import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# 从 scripts/ 往上是 carebot_nav_kit/ ，再往上是 src/
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)
NODES = os.path.join(PKG, 'nodes')
for p in (HERE, NODES, SRC):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim_arena  # noqa: E402


# ---------------------------------------------------------------------------
# 输出编码
# ---------------------------------------------------------------------------
# ⚠️ 这个脚本**打印中文**（报告全中文），但原来**完全没做编码设置**。
#    在没有 UTF-8 locale 的环境下（`rosrun` 就是），Python 2.7 默认
#    ASCII 编码，直接崩：
#        UnicodeEncodeError: 'ascii' codec can't encode characters
#    而崩的位置在报告开头，看起来像"脚本没输出"，很容易误判。
#
# 用项目里唯一那份实现（`utf8_stdio`），找不到就退化成**字节串容忍版**
# （不能只写 codecs.getwriter —— 那会把 traceback 的字节串写崩）。
try:
    from utf8_stdio import setup_all as setup_stdout
    setup_stdout()
except ImportError:
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


def main():
    grid = sim_arena.build_grid()
    h = len(grid)
    w = max(len(r) for r in grid)
    res = sim_arena.RESOLUTION
    print('')
    print('  Arena: %d rows x %d cols @ %.3f m = %.1f x %.1f m'
          % (h, w, res, w * res, h * res))

    # ---- 1. row order ----
    print('')
    print('  [1] sim_arena row order')
    print('      ARENA[0]   = %s' % ''.join(grid[0][:46]))
    print('      ARENA[h-1] = %s' % ''.join(grid[h - 1][:46]))
    open_rows = [i for i, row in enumerate(grid)
                 if row.count('.') > w * 0.9]
    print('      fully-open rows: %s' % open_rows)
    if open_rows and max(open_rows) >= h - 3:
        print('      -> opening is in the LAST rows:'
              ' row 0 = north, row h-1 = south')
    else:
        print('      -> opening is in the FIRST rows: row 0 = south')

    # ---- 2. FakeWorld ----
    print('')
    print('  [2] FakeWorld.is_occupied (the mapping the sim really uses)')
    try:
        from fake_sensors import FakeWorld
    except Exception as exc:
        print('      import fake_sensors failed: %s'
              % str(exc).decode('utf-8', 'replace')
              if not isinstance(exc, str) else str(exc))
        return 1
    fw = FakeWorld([''.join(r) for r in grid], res=res)

    def fw_cell(wx, wy):
        """复刻修复后的映射：Y 轴翻转。"""
        gx = int((wx - fw.ox) / fw.res)
        gy = fw.h - 1 - int((wy - fw.oy) / fw.res)
        return gx, gy

    bad = []
    print('      %-14s %-18s %-12s %s'
          % ('point', 'world', 'cell', 'free?'))
    for name, v in sim_arena.POINTS.items():
        x, y = v[0], v[1]
        gx, gy = fw_cell(x, y)
        occ = fw.is_occupied(x, y)
        direct = (0 <= gy < h and 0 <= gx < w and grid[gy][gx] == '#')
        ok = (not occ) and (not direct)
        if not ok:
            bad.append(name)
        print('      %-14s (%+.2f,%+.2f)   (%2d,%2d)   %s%s'
              % (name, x, y, gx, gy,
                 'FREE' if ok else 'WALL',
                 '' if occ == direct else '  <- paths disagree!'))

    # ---- 3. raycast from start ----
    print('')
    print('  [3] raycast from start'
          ' (open field; short toward the arena opening is expected)')
    sx, sy = sim_arena.POINTS['start'][0], sim_arena.POINTS['start'][1]
    # ⚠️ 判据**不能**是"每个方向都 >1 m"。
    #
    # 起点就在场地**南侧开口**附近，朝正南（270°）只看到约 0.8 m
    # 是**正确**的 —— 开口只有两行深（0.2 m 走廊 + 外墙）。
    # 原来那条"<1 m 就报警"的判据过严，会**误报**；
    # 我第一次看到 FAIL 时差点又去查 Y 轴映射。
    #
    # 真正该报警的是"近到会撞上"：车体半径 0.18 + 膨胀 0.22 = 0.40 m，
    # 所以取 0.5 m 作为警戒线。
    short = []
    for deg in range(0, 360, 45):
        d = fw.raycast(sx, sy, math.radians(deg), 12.0)
        flag = ''
        if d < 0.5:
            flag = '  <- TOO CLOSE (would collide)'
            short.append(deg)
        elif d < 1.0:
            flag = '  (short, but ok - near the arena opening)'
        print('      %3d deg  %6.2f m%s' % (deg, d, flag))

    # ---- 4. conclusion ----
    print('')
    print('  ===== CONCLUSION =====')
    if bad:
        print('  FAIL: %d points inside a wall: %s'
              % (len(bad), ', '.join(bad)))
    else:
        print('  OK: all points on free cells')
    if short:
        print('  FAIL: %d directions from start < 1 m (%s)'
              ' -- Y mapping still wrong' % (len(short), short))
    else:
        print('  OK: start is in the open, raycast sane')
    if not bad and not short:
        print('  OK: Y mapping self-consistent, sim geometry trustworthy')
    print('')
    return 0 if (not bad and not short) else 1


if __name__ == '__main__':
    sys.exit(main())
