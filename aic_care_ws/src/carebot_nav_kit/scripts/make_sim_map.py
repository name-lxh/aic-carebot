#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
make_sim_map.py — 生成与 fake_sensors 世界一致的测试地图

为什么需要
----------
端到端测试要求**地图与仿真世界一致**：
    · 地图尺寸决定了 amcl 的定位范围
    · 地图上的障碍位置决定了 costmap 的静态层
    · 假传感器的激光是从同一个世界算出来的

如果两者不一致（比如地图 4m×4m 而世界 8m×8m），会出现：
    · 机器人在"地图外"移动，amcl 无法收敛
    · costmap 静态层与实际障碍错位，规划出穿墙的路径

注意：这是**测试用地图**，不是比赛场地地图。
      比赛场地地图要在现场用 gmapping 建（见 建图与导航操作手册.md）。

用法
----
    python3 make_sim_map.py                        # 默认输出到 maps/
    python3 make_sim_map.py --out /tmp/sim
"""

import argparse
import io
import os
import sys


def setup_stdout():
    """让 stdout/stderr 能输出中文 —— Python 2.7 / 3.6 / 3.11 都能用。

    ⚠️ 这段代码修正了 4 次才对，动它之前请看完下面每一条：

    1) 不用 `hasattr(sys.stdout, 'reconfigure')` 判断 —— 那是 3.7+ 的方法。
       目标环境 Ubuntu 18.04 是 Python 3.6，hasattr 为假，设置被静默跳过，
       随后打印中文抛 UnicodeEncodeError。

    2) 不能只依赖 `stream.buffer` —— 那是 Python 3 的属性。
       Python 2.7（ROS Melodic 的实际运行版本）的 sys.stdout 没有它，
       getattr 得 None，同样静默跳过。

    3) 用 `codecs.getwriter` 包 Python 2 的 sys.stdout 时，**必须**处理
       字节串输入。因为 Python 2 里有两类代码往 stderr 写**字节串**：

           traceback.print_exc()     → file.write(str)  字节串
           logging.StreamHandler     → stream.write(msg + "\n")

       标准 StreamWriter 只接受 unicode，收到字节串会先 decode('ascii')，
       遇到非 ASCII 就崩：
           UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5
       后果是"为了看清异常而打印异常时崩掉"，真正的错误彻底看不到。
       —— 本项目实际踩过这个坑，排查了很久。

    4) 所以这里自定义 _Utf8Writer：字节串原样写，unicode 编码后写。

    实测环境：Python 3.11（本地开发）/ 3.6.9（Ubuntu 18.04 默认）
              / 2.7.17（ROS Melodic 跑节点的实际版本）
    """
    import codecs as _codecs

    class _Utf8Writer(_codecs.StreamWriter):
        """字节串与 unicode 都接受，都往底层流写 UTF-8 字节。"""

        def write(self, s):
            if s is None:
                return
            try:
                if isinstance(s, bytes):
                    # 已经是字节串（traceback / logging 会这样写）
                    self.stream.write(s)
                    return
                # unicode → UTF-8 字节
                self.stream.write(s.encode('utf-8', 'backslashreplace'))
            except Exception:
                # 兜底：尽全力输出，绝不因为编码问题让程序崩
                try:
                    self.stream.write(repr(s))
                except Exception:
                    pass

    def _wrap(stream):
        if stream is None:
            return stream
        # 已经是 UTF-8 的，不动
        enc = (getattr(stream, 'encoding', None) or '').lower().replace('-', '')
        if enc in ('utf8', 'utf_8'):
            return stream
        # 优先用底层字节流（Python 3）；Python 2 没有 .buffer，直接用流本身
        buf = getattr(stream, 'buffer', None)
        if buf is None:
            buf = stream
        try:
            return _Utf8Writer(buf, errors='backslashreplace')
        except Exception:
            pass
        # 退化：Python 3.7+ 的 reconfigure
        reconf = getattr(stream, 'reconfigure', None)
        if callable(reconf):
            try:
                reconf(encoding='utf-8', errors='backslashreplace')
            except Exception:
                pass
        return stream

    sys.stdout = _wrap(sys.stdout)
    sys.stderr = _wrap(sys.stderr)


# 与 carebot_nav_kit/nodes/fake_sensors.py 的 DEFAULT_MAP 保持一致
# 改了那边就要改这里（test_pure_logic.py 会校验一致性）
SIM_MAP = [
    "##########################################",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#.........####...........................",
    "#.........####...........................",
    "#.........####...........................",
    "#.........####...........................",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#....................######..............",
    "#....................######..............",
    "#....................######..............",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "#........................................#",
    "##########################################",
]

RESOLUTION = 0.2       # 与 FakeWorld 的 res 一致


def main():
    setup_stdout()

    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=None, help=u'输出目录')
    ap.add_argument('--name', default='sim_map', help=u'地图文件名（不含扩展名）')
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.out or os.path.abspath(os.path.join(here, '..', 'maps'))
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    h = len(SIM_MAP)
    w = max(len(r) for r in SIM_MAP)
    rows = [r.ljust(w, '#') for r in SIM_MAP]

    # PGM：P5 二进制灰度。ROS map_server 的约定：
    #   0    = 占据（黑）
    #   254  = 空闲（白）
    #   205  = 未知（灰）
    # ⚠️ PGM 的行序是**从上到下**，而 ROS 地图的 origin 在**左下角**，
    #    所以写入时要上下翻转。这个坑不注意会导致地图整体镜像。
    pgm_path = os.path.join(out_dir, args.name + '.pgm')
    with open(pgm_path, 'wb') as fp:
        fp.write(b'P5\n')
        fp.write(b'# AIC carebot_nav_kit simulation map\n')
        fp.write(('%d %d\n' % (w, h)).encode('ascii'))
        fp.write(b'255\n')
        for y in range(h - 1, -1, -1):          # 翻转
            line = rows[y]
            for x in range(w):
                fp.write(b'\x00' if line[x] == '#' else b'\xfe')

    # YAML
    ox = -w * RESOLUTION / 2.0
    oy = -h * RESOLUTION / 2.0
    yaml_path = os.path.join(out_dir, args.name + '.yaml')
    with io.open(yaml_path, 'w', encoding='utf-8', newline='\n') as fp:
        fp.write(u'''# AIC carebot_nav_kit 仿真测试地图
# 由 scripts/make_sim_map.py 生成，与 nodes/fake_sensors.py 的世界一致
#
# ⚠️ 这是**测试用地图**，不是比赛场地地图。
#    比赛场地地图要用 gmapping 现场建（见 docs/建图与导航操作手册.md）
image: %s.pgm
resolution: %.3f
origin: [%.3f, %.3f, 0.000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
''' % (args.name, RESOLUTION, ox, oy))

    n_wall = sum(r.count('#') for r in rows)
    print(u'已生成仿真地图：')
    print(u'  %s' % pgm_path)
    print(u'  %s' % yaml_path)
    print(u'  尺寸      ：%d × %d 格 = %.1f × %.1f m'
          % (w, h, w * RESOLUTION, h * RESOLUTION))
    print(u'  分辨率    ：%.3f m/格' % RESOLUTION)
    print(u'  原点      ：(%.3f, %.3f)' % (ox, oy))
    print(u'  障碍格数  ：%d' % n_wall)
    print()
    print(u'用法：')
    print(u'  rosrun map_server map_server %s' % yaml_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
