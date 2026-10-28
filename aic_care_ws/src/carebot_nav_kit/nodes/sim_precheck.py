#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
sim_precheck.py — 仿真启动前的自检：地图与点位必须对得上

## 为什么需要这个

仿真里最容易出、又最难查的一类问题是**坐标系对不上**：

    · 地图 yaml 的 `origin` 写错了
      → map 坐标系与 `fake_sensors` 的世界坐标差一个偏移量
    · 点位（waypoints）落在墙里或者地图外
      → move_base 报"目标点不可达"，但不说为什么
    · 点位坐标系搞错了（世界坐标当成了地图像素坐标）
      → 表现是"导航说到了，但机器人位置完全不对"

这三种**单独看每一份文件都是正常的**：

    · sim_arena.yaml 语法没问题
    · waypoints_sim.yaml 格式没问题
    · fake_sensors.py 跑得好好的

所以只能**交叉比对**才能发现。这个节点就是干这个的。

## 检查什么

  1. 地图 yaml 能被解析，`origin` 是不是 `(-w*res/2, -h*res/2)`
     —— 这是 `fake_sensors.FakeWorld`（中心为原点）的约定
  2. 每个点位是否落在**可通行格**上（不是墙里、不是地图外）
  3. 点位是否在地图范围内
  4. 点位配置里必需的 5 个点（qr_area/point_A/room_B/C/D）都有

## 为什么独立成节点而不是塞进 launch

launch 里只能做 XML 和 `$(eval ...)` 能表达的事，
读 PGM 像素、判断"这个格子是不是墙"必须在代码里做。

用法（通常由 sim_arena.launch 带起来，也可单独跑）：
    rosrun carebot_nav_kit sim_precheck.py \
        _map_file:=.../sim_arena.yaml \
        _waypoints_file:=.../waypoints_sim.yaml
"""

import io
import os
import sys
import traceback

import rospy


# ---------------------------------------------------------------------------
# 输出编码（Python 2.7 上 stdout 是 ASCII，不处理的话中文直接崩）
# ---------------------------------------------------------------------------
def setup_stdout():
    """让 stdout/stderr 能输出中文，且**容忍字节串**输入。

    ## 为什么不能直接用 codecs.getwriter

    Py2 里有两类代码往流里写**字节串**：
        traceback.print_exc()    → file.write(str)  字节串
        logging.StreamHandler    → stream.write(msg + "\\n")

    而 `codecs.getwriter` 返回的标准 StreamWriter **只接受 unicode**，
    收到字节串会先 `decode('ascii')`：
        UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5

    后果是"**为了看清异常而打印 traceback 时反而崩掉**"，
    真正的错误彻底看不到。本项目实际踩过，排查了很久。

    所以要自定义一个继承 `codecs.StreamWriter`、但覆写 `write`
    以容忍 bytes 的 writer。

    ## 优先复用项目里唯一那份实现

    `scripts/utf8_stdio.py` 就是这份实现（`utf8_stdio._Utf8Writer`）。
    `nodes/` 和 `scripts/` 是两个目录，无法直接 import，
    所以这里先把 `../scripts` 加进 sys.path 再试；
    找不到就退化成等价的最小实现。

    ⚠️ **编码处理只应有一份权威实现** ——
       本项目一度有 5 处内联副本，其中还出现过
       "对已被包过的流再包一层 → UnicodeDecodeError" 的事故。
    """
    import os as _os

    here = _os.path.dirname(_os.path.abspath(__file__))
    for cand in (_os.path.join(here, '..', 'scripts'),
                 _os.path.join(here, '..', '..', 'scripts')):
        cand = _os.path.abspath(cand)
        if _os.path.isdir(cand) and cand not in sys.path:
            sys.path.insert(0, cand)
    try:
        from utf8_stdio import setup_all
        setup_all()
        return
    except ImportError:
        pass

    # 退化：等价的最小实现（**仍然容忍字节串**）
    import codecs

    class _Writer(codecs.StreamWriter):
        def write(self, s):
            if s is None:
                return
            try:
                if isinstance(s, bytes):
                    self.stream.write(s)
                    return
                self.stream.write(s.encode('utf-8', 'backslashreplace'))
            except Exception:
                try:
                    self.stream.write(repr(s))
                except Exception:
                    pass

    for name in ('stdout', 'stderr'):
        st = getattr(sys, name, None)
        if st is None:
            continue
        try:
            buf = getattr(st, 'buffer', st)
            setattr(sys, name, _Writer(buf, errors='backslashreplace'))
        except Exception:
            pass


def _plain_yaml(path):
    """极简 YAML 读取 —— 只处理 `key: value` 和 `[{...}]` 行。

    ⚠️ 为什么不用 `yaml` 模块：
       ROS 的 Python 2 环境里 pyyaml 一般有，但我们这个节点要能在
       "刚装完 ROS、还没装齐依赖"的情况下跑 —— 自检节点本身
       依赖越少越好，不然它会跟着一起失败，就失去自检的意义了。

       实际用到的键只有 image/resolution/origin 和 waypoints 里的
       一行行坐标，手写解析完全够，而且不会有依赖问题。
    """
    out = {}
    with io.open(path, encoding='utf-8-sig', errors='replace') as fp:
        for line in fp:
            s = line.strip()
            if not s or s.startswith('#') or s.startswith('-'):
                continue
            if ':' not in s:
                continue
            k, _, v = s.partition(':')
            out[k.strip()] = v.strip()
    return out


def parse_waypoints(path):
    """解析 waypoints yaml，返回 {name: (x, y, yaw)}。

    格式约定（见 config/waypoints_sim.yaml）：
        name: {x: -1.30, y: 0.40, yaw: 0.0000}
    """
    pts = {}
    with io.open(path, encoding='utf-8-sig', errors='replace') as fp:
        for line in fp:
            s = line.strip()
            if not s or s.startswith('#') or '{' not in s:
                continue
            name, _, rest = s.partition(':')
            name = name.strip()
            if not rest.strip().startswith('{'):
                continue
            body = rest.strip().strip('{}')
            kv = {}
            for part in body.split(','):
                if ':' not in part:
                    continue
                k, _, v = part.partition(':')
                try:
                    kv[k.strip()] = float(v.strip())
                except ValueError:
                    pass
            if 'x' in kv and 'y' in kv:
                pts[name] = (kv['x'], kv['y'], kv.get('yaw', 0.0))
    return pts


def read_pgm_header(path):
    """读 PGM 的 P5 头，返回 (w, h) 或 None。

    只读头，不读全部像素 —— 但为了判断"某格是不是墙"还是得读全。
    """
    with open(path, 'rb') as fp:
        blob = fp.read()

    # 解析 P5 头：魔数 / 宽 / 高 / 最大灰度，中间可以有 # 注释
    i = 0
    tok = []
    while len(tok) < 4:
        while i < len(blob) and blob[i:i + 1].isspace():
            i += 1
        if i >= len(blob):
            return None
        if blob[i:i + 1] == b'#':
            while i < len(blob) and blob[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(blob) and not blob[j:j + 1].isspace():
            j += 1
        tok.append(blob[i:j])
        i = j
    i += 1        # 跳掉最大灰度后的那个空白字符

    try:
        if tok[0] != b'P5':
            return None
        w, h = int(tok[1]), int(tok[2])
    except Exception:
        return None
    data = blob[i:]
    if len(data) < w * h:
        return ('SHORT', w, h, len(data))
    return (w, h, data)


class Precheck(object):

    def __init__(self):
        self.errors = []
        self.warns = []
        self.oks = []

    def err(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warns.append(msg)

    def ok(self, msg):
        self.oks.append(msg)

    # -- 各项检查 ---------------------------------------------------------
    def check_map(self, map_file):
        if not os.path.isfile(map_file):
            self.err(u'地图文件不存在：%s' % map_file)
            return None
        self.ok(u'地图文件存在：%s' % os.path.basename(map_file))

        meta = _plain_yaml(map_file)
        try:
            res = float(meta.get('resolution', '0'))
        except ValueError:
            res = 0.0
        if res <= 0:
            # ⚠️ 不能用 %r：%r 产出**字节串**，插进 unicode 格式串
            #    时 Py2 会按 ASCII 解码它 —— 内容含中文就崩。
            #    resolution 通常读不到（走到这个分支），
            #    但 res 非法时 meta 里的值可能是任何字符串。
            self.err(u'地图 resolution 读不到或非法：%s'
                     % to_text(meta.get('resolution')))
            return None

        # origin: [x, y, yaw]
        origin = meta.get('origin', '')
        try:
            ox, oy = [float(v.strip()) for v in
                      origin.strip().strip('[]').split(',')[:2]]
        except Exception:
            # ⚠️ 同上：origin 若含非 ASCII 就会崩（见上一条说明）
            self.err(u'地图 origin 解析失败：%s' % to_text(origin))
            return None

        img = meta.get('image', '').strip()
        pgm = img if os.path.isabs(img) else \
            os.path.join(os.path.dirname(map_file), img)
        if not os.path.isfile(pgm):
            self.err(u'地图引用的图片不存在：%s' % pgm)
            return None

        hdr = read_pgm_header(pgm)
        if hdr is None:
            self.err(u'PGM 头解析失败：%s' % pgm)
            return None
        if hdr[0] == 'SHORT':
            self.err(u'PGM 数据长度不足：期望 %d 字节，实际 %d'
                     % (hdr[1] * hdr[2], hdr[3]))
            return None
        w, h, data = hdr

        self.ok(u'地图 %d×%d 格 @ %.3f m/格 = %.1f m × %.1f m'
                % (w, h, res, w * res, h * res))

        # ★ 核心检查：origin 必须等于 (-w*res/2, -h*res/2)
        ex = -w * res / 2.0
        ey = -h * res / 2.0
        tol = res / 2.0        # 半个格子以内可接受
        if abs(ox - ex) > tol or abs(oy - ey) > tol:
            self.err(
                u'★ 地图 origin 与 fake_sensors 的世界坐标不匹配！\n'
                u'      文件里写的是  [%.3f, %.3f]\n'
                u'      应该是        [%.3f, %.3f]\n'
                u'      原因：fake_sensors.FakeWorld 用**中心为原点**的\n'
                u'            世界坐标（ox = -w*res/2, oy = -h*res/2）。\n'
                u'            origin 写错 → map 坐标系整体偏移，\n'
                u'            表现是"导航说到了，但机器人位置完全不对"。\n'
                u'      修法：跑 scripts/sim_arena.py --write 重新生成地图'
                % (ox, oy, ex, ey))
        else:
            self.ok(u'地图 origin [%.3f, %.3f] 与仿真世界一致'
                    % (ox, oy))

        return (w, h, res, ox, oy, data)

    def check_waypoints(self, wp_file, mapinfo):
        if not os.path.isfile(wp_file):
            self.err(u'点位文件不存在：%s' % wp_file)
            return
        pts = parse_waypoints(wp_file)
        if not pts:
            self.err(u'点位文件里没解析出任何点位：%s' % wp_file)
            return
        self.ok(u'点位文件解析出 %d 个点：%s'
                % (len(pts), u', '.join(sorted(pts))))

        # 必需点位
        need = ['qr_area', 'point_A', 'room_B', 'room_C', 'room_D']
        miss = [n for n in need if n not in pts]
        if miss:
            self.err(u'缺少必需点位：%s' % u', '.join(miss))
        else:
            self.ok(u'5 个必需点位都在（qr_area / point_A / room_B/C/D）')

        if not mapinfo:
            self.warn(u'地图没解析成功，跳过点位可达性检查')
            return

        w, h, res, ox, oy, data = mapinfo

        for name in sorted(pts):
            x, y, _yaw = pts[name]
            gx = int((x - ox) / res)
            # ⚠️ 网格第 0 行在最上面，世界 +y 向上 → 必须翻转
            gy = h - 1 - int((y - oy) / res)
            if not (0 <= gx < w and 0 <= gy < h):
                self.err(u'点位 %-12s (%.2f, %.2f) 落在地图外'
                         u'（格 %d,%d，地图 %d×%d）'
                         % (name, x, y, gx, gy, w, h))
                continue
            val = data[gy * w + gx]
            if val < 100:
                self.err(u'点位 %-12s (%.2f, %.2f) 落在墙/障碍里'
                         u'（格 %d,%d 灰度=%d）—— 导航永远到不了'
                         % (name, x, y, gx, gy, val))
            else:
                self.ok(u'点位 %-12s (%.2f, %.2f) 在可通行格'
                        % (name, x, y))


def main():
    setup_stdout()
    rospy.init_node('sim_precheck', anonymous=False, log_level=rospy.INFO)

    map_file = rospy.get_param('~map_file', '')
    wp_file = rospy.get_param('~waypoints_file', '')

    pc = Precheck()
    print(u'')
    print(u'=' * 70)
    print(u'  仿真启动自检（地图 + 点位一致性）')
    print(u'=' * 70)

    mapinfo = pc.check_map(map_file) if map_file else None
    if not map_file:
        pc.err(u'没给 ~map_file 参数')

    if wp_file:
        pc.check_waypoints(wp_file, mapinfo)
    else:
        pc.err(u'没给 ~waypoints_file 参数')

    # ---- 输出 ----
    if pc.oks:
        print(u'')
        for m in pc.oks:
            print(u'  [OK]   %s' % m)
    if pc.warns:
        print(u'')
        for m in pc.warns:
            print(u'  [警告] %s' % m)
    if pc.errors:
        print(u'')
        for m in pc.errors:
            print(u'  [错误] %s' % m)

    print(u'')
    print(u'=' * 70)
    if pc.errors:
        print(u'  ✘ 自检发现 %d 个问题 —— 仿真结果将不可信' % len(pc.errors))
        print(u'    （导航可能"成功"但位置是错的，或者目标点永远到不了）')
        print(u'=' * 70)
        print(u'')
        # ⚠️ 这里**只报警不退出** —— launch 里 required=false，
        #    而且用户可能就想看看错误环境下的表现。但要吵得够响。
        #
        # ⚠️⚠️ 下面这句日志**必须是 ASCII** —— 本项目踩最多次的坑。
        #
        # 原来写的是 rospy.logerr(u'仿真自检失败…')，Python 2.7 下必崩：
        #     rosgraph/roslogging.py:249
        #         msg = msg.replace('${message}', str(record_message))
        #     UnicodeEncodeError: 'ascii' codec can't encode characters
        #
        # 而这行**正好在"自检失败"的分支里** —— 一崩就把自检结果也吞了，
        # 表现成"自检跑完什么都没说"，完全看不出是日志编码问题。
        # （单元测试 test_rospy_logging_nodes_have_fix 抓到的）
        #
        # 约定：中文说明放注释，日志只放 ASCII。
        rospy.logerr('[sim_precheck] self-check FAILED, '
                     'see [ERROR] lines above')
    else:
        print(u'  ✔ 自检通过：地图与点位一致，仿真结果可信')
        print(u'=' * 70)
    print(u'')

    # 一次性节点，检查完就退
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
    except Exception:
        traceback.print_exc()
        sys.exit(1)
