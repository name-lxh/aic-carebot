#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
make_placeholder_map.py — 生成占位地图（用于导航栈自检）

用途
----
`map_server` 加载不到地图会直接启动失败，导致 navigation.launch 整个起不来 ——
即使问题只是"还没建图"。这对第一次上手的人很不友好，也让人误以为导航栈坏了。

本脚本生成一张 **3m×3m 的空场地占位地图**（四周有墙），让：
  · navigation.launch 能正常启动
  · amcl / move_base / costmap 能加载并跑起来
  · 团队可以在建图之前先把整条链路验证一遍

⚠️ 占位地图**不能用于正式导航**。它是空房间，与真实场地不符。
   建好真实地图后，用 save_map.sh 覆盖 maps/mymap.yaml 即可。

用法
----
    python3 make_placeholder_map.py                    # 写到 ../maps/
    python3 make_placeholder_map.py --out /tmp/x.yaml  # 指定输出
    python3 make_placeholder_map.py --size 3.0 --wall 0.1
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


def write_pgm(path, width, height, pixels):
    """写二进制 PGM（P5）。map_server 支持 P5。"""
    with open(path, 'wb') as fp:
        fp.write(b'P5\n')
        fp.write(('# %s\n' % 'placeholder map - NOT for real navigation').encode('ascii'))
        fp.write(('%d %d\n' % (width, height)).encode('ascii'))
        fp.write(b'255\n')
        fp.write(bytes(bytearray(pixels)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=None, help='输出 yaml 路径')
    ap.add_argument('--name', default='mymap', help='地图名')
    ap.add_argument('--size', type=float, default=3.0, help='场地边长（米）')
    ap.add_argument('--resolution', type=float, default=0.05, help='分辨率 m/像素')
    ap.add_argument('--wall', type=float, default=0.15, help='墙体厚度（米）')
    ap.add_argument('--margin', type=float, default=0.5, help='地图四周留白（米）')
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    out_dir = os.path.dirname(args.out) if args.out else os.path.join(pkg, 'maps')
    out_yaml = args.out or os.path.join(out_dir, args.name + '.yaml')
    out_pgm = os.path.join(out_dir, args.name + '.pgm')

    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    res = args.resolution
    total = args.size + 2 * args.margin
    n = int(round(total / res))

    wall_px = max(1, int(round(args.wall / res)))
    margin_px = int(round(args.margin / res))

    # 像素值语义（map_server）：
    #   0   = 障碍（黑）
    #   254 = 自由（白）
    #   205 = 未知（灰）
    FREE = 254
    OCC = 0
    # 地图整体先填"自由"，只在四周画墙
    pixels = bytearray([FREE]) * (n * n)

    for y in range(n):
        for x in range(n):
            on_border = (
                x < margin_px or x >= n - margin_px or
                y < margin_px or y >= n - margin_px
            )
            # 墙体：紧贴场地边缘往里 wall_px 厚
            in_wall = (
                (margin_px <= x < margin_px + wall_px) or
                (n - margin_px - wall_px <= x < n - margin_px) or
                (margin_px <= y < margin_px + wall_px) or
                (n - margin_px - wall_px <= y < n - margin_px)
            )
            if in_wall:
                pixels[y * n + x] = OCC
            elif on_border:
                pixels[y * n + x] = 205      # 未知
            else:
                pixels[y * n + x] = FREE

    write_pgm(out_pgm, n, n, pixels)

    # 原点：让地图中心对应 (0,0) 附近的场地
    # map_server 的 origin 是"地图左下角像素在地图坐标系中的位置"
    origin_x = round(-(args.margin + args.size / 2.0), 6)
    origin_y = round(-(args.margin + args.size / 2.0), 6)

    yaml_text = u'''# ============================================================================
#  占位地图 —— 由 make_placeholder_map.py 生成
# ============================================================================
#  ⚠️⚠️ 这张地图是**空房间**，仅供导航栈自检使用，不能用于正式比赛导航！
#
#  它的作用：让 map_server 能正常加载，从而让 navigation.launch 完整启动，
#            便于在真正建图之前先验证 amcl / move_base / costmap 的配置是否正确。
#
#  生成参数：
#    场地     %s m × %s m
#    分辨率   %s m/像素
#    图像     %d × %d 像素
#    墙厚     %s m
#    留白     %s m
#
#  替换方法（建好真实地图后）：
#    roslaunch carebot_nav_kit mapping.launch      # 建图
#    rosrun carebot_nav_kit save_map.sh -n mymap   # 保存，会覆盖本文件
# ============================================================================

image: %s.pgm
resolution: %s
origin: [%s, %s, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
''' % (args.size, args.size, res, n, n, args.wall, args.margin,
       args.name, res, origin_x, origin_y)

    with io.open(out_yaml, 'w', encoding='utf-8') as fp:
        fp.write(yaml_text)

    print(u'已生成占位地图：')
    print(u'  %s' % out_pgm)
    print(u'  %s' % out_yaml)
    print(u'  图像 %d×%d 像素，分辨率 %.3f m，覆盖 %.1f×%.1f m'
          % (n, n, res, n * res, n * res))
    print()
    print(u'⚠️  这是空房间占位图，仅供自检。正式导航前必须用真实地图替换。')
    return 0


if __name__ == '__main__':
    setup_stdout()
    try:
        sys.exit(main())
    except Exception as exc:
        print('生成失败：%s' % repr(exc))
        sys.exit(1)
