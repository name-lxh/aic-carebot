#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
verify_nav_config.py — 导航配置一致性离线校验

用途
----
在**没有机器人**的情况下，校验 carebot_nav_kit 的配置文件之间是否自洽。
这些是最容易出错、也最难在实车上定位的问题：

  1. robot_radius 在 base.yaml 与 costmap 配置里是否一致
  2. 坐标系命名是否统一（base_footprint 还是 base_link）
  3. 雷达滤波的截断距离是否大于车体半径（否则"把自己当障碍"）
  4. EKF 协方差矩阵是否合法的 15×15
  5. 话题名是否与上游发布的匹配（/scan_filtered 有没有人发布）
  6. 所有被 launch 引用的文件是否真实存在
  7. max_vel 是否满足赛题规则下限
  8. DWA 的 holonomic_robot 是否与底盘类型匹配

用法
----
    python3 verify_nav_config.py [--pkg-dir <carebot_nav_kit 目录>]
"""

import argparse
import io
import os
import re
import sys

try:
    import yaml
except ImportError:
    print('需要 pyyaml：pip install pyyaml')
    sys.exit(2)


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

OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'

# ⚠️⚠️ 颜色常量必须是 **unicode**（u'' 前缀），不能是字节串。
#
# 这是本项目第三次踩同一类 bug：**Py2 里把字节串插进 unicode 格式串，
# 会先按 ASCII 解码那个字节串**：
#
#     >>> u'%s' % '\033[0m'
#     UnicodeDecodeError: 'ascii' codec can't decode byte 0x1b
#
# 也就是说 **出错的是"混合编码"，不是"中文"** —— 颜色串本身是纯 ASCII，
# 看着人畜无害，所以特别难往它身上想。
#
# 实测崩在这里：
#     print(u'\033[36;1m── 1. 车体几何一致性%s' % R)
#                  ↑ 字节串字面量   ↑ 中文        ↑ R 也是字节串
# 报错信息是
#     UnicodeDecodeError: 'ascii' codec can't decode byte 0xe4
# 指向的却是中文那一列（0xe4 是"─"的首字节），**完全误导**。
#
# 改法：颜色常量统一加 u 前缀。
C = {OK: u'\033[32;1m', WARN: u'\033[33;1m', FAIL: u'\033[31;1m'}
R = u'\033[0m'

results = []


def chk(level, name, detail='', fix=''):
    results.append((level, name, detail, fix))
    print(u'  %s[%s]%s %-44s %s' % (C[level], level, R, name, detail))
    if fix and level != OK:
        print(u'         → %s' % fix)


def load(path):
    with open(path, "rb") as fp:
        return yaml.safe_load(fp.read())


def main():
    setup_stdout()
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkg-dir', default=None)
    args = ap.parse_args()

    pkg = args.pkg_dir
    if pkg is None:
        here = os.path.dirname(os.path.abspath(__file__))
        pkg = os.path.dirname(here)   # 从 scripts/ 上跳一级

    if not os.path.isfile(os.path.join(pkg, 'package.xml')):
        print('找不到 carebot_nav_kit 包目录：%s' % pkg)
        return 2

    print(u'\n\033[36;1m')
    print(u'=' * 74)
    print(u'  carebot_nav_kit 配置一致性校验')
    print(u'  包目录：%s' % pkg)
    print(u'=' * 74)
    print(R)

    cfg = os.path.join(pkg, 'config')

    # ------------------------------------------------------------------
    # 加载配置
    # ------------------------------------------------------------------
    try:
        base = load(os.path.join(cfg, 'robot', 'base.yaml'))
        costmap_common = load(os.path.join(cfg, 'nav', 'costmap_common_params.yaml'))
        gcost = load(os.path.join(cfg, 'nav', 'global_costmap_params.yaml'))
        lcost = load(os.path.join(cfg, 'nav', 'local_costmap_params.yaml'))
        amcl = load(os.path.join(cfg, 'nav', 'amcl_params.yaml'))
        dwa = load(os.path.join(cfg, 'nav', 'dwa_local_planner_params.yaml'))
        gplanner = load(os.path.join(cfg, 'nav', 'global_planner_params.yaml'))
        gmapping = load(os.path.join(cfg, 'slam', 'gmapping_params.yaml'))
        ekf = load(os.path.join(cfg, 'nav', 'ekf_params.yaml'))
        lfilter = load(os.path.join(cfg, 'filters', 'laser_filter_chain.yaml'))
    except Exception as exc:
        print('配置加载失败：%s' % repr(exc))
        return 2

    rp = base['robot']

    # ==================================================================
    print(u'\033[36;1m── 1. 车体几何一致性%s' % R)

    r_base = float(rp['geometry']['radius'])
    r_cost = float(costmap_common.get('robot_radius', -1))
    if abs(r_base - r_cost) < 1e-6:
        chk(OK, 'robot_radius 一致', '%.3f m' % r_base)
    else:
        chk(FAIL, 'robot_radius 不一致',
            'base.yaml=%.3f, costmap=%.3f' % (r_base, r_cost),
            '两处必须相同，否则膨胀层与实际车体不符')

    # 激光近端截断必须 > 车体半径
    lower = None
    for f in lfilter.get('scan_filter_chain', []):
        if f.get('name') == 'range':
            lower = float(f['params']['lower_threshold'])
    if lower is None:
        chk(WARN, '未找到距离滤波器', '', '激光会扫到车体自身')
    elif lower <= r_base:
        chk(FAIL, '激光截断距离 ≤ 车体半径',
            'lower=%.3f, radius=%.3f' % (lower, r_base),
            '把 lower_threshold 提到 %.2f 以上，否则机器人把自己当障碍' % (r_base + 0.04))
    else:
        chk(OK, '激光截断距离 > 车体半径',
            'lower=%.3f > radius=%.3f（余量 %.3f m）'
            % (lower, r_base, lower - r_base))

    # inflation_radius 应略大于车体半径
    infl = float(costmap_common['inflation_layer']['inflation_radius'])
    if infl < r_base:
        chk(FAIL, 'inflation_radius < 车体半径',
            'infl=%.3f < radius=%.3f' % (infl, r_base),
            '膨胀半径小于车体半径会导致贴着障碍规划，容易蹭到')
    else:
        chk(OK, 'inflation_radius ≥ 车体半径',
            'infl=%.3f（余量 %.3f m）' % (infl, infl - r_base))

    # ==================================================================
    print(u'\n\033[36;1m── 2. 坐标系命名统一性%s' % R)

    fr = rp['frames']
    checks = [
        ('base_footprint', fr['base_footprint'],
         gcost['global_costmap']['robot_base_frame']),
        ('base_footprint(costmap_local)', fr['base_footprint'],
         lcost['local_costmap']['robot_base_frame']),
        ('base_footprint(amcl)', fr['base_footprint'], amcl['base_frame_id']),
        ('base_footprint(gmapping)', fr['base_footprint'], gmapping['base_frame']),
        ('base_footprint(ekf)', fr['base_footprint'], ekf['base_link_frame']),
    ]
    mismatch = [c for c in checks if c[1] != c[2]]
    if not mismatch:
        chk(OK, 'base_frame 全部统一为 base_footprint',
            '%d 处配置一致' % len(checks))
    else:
        for name, want, got in mismatch:
            chk(FAIL, 'base_frame 不一致：%s' % name,
                '期望 %s，实际 %s' % (want, got),
                'base_link 与 base_footprint 混用会让定位与规划错位')

    # odom frame
    # 注意：这里刻意不用海象运算符 :=（Python 3.8+ 才支持）。
    # 目标机器可能是 Ubuntu 18.04 + Python 3.6，用 := 会直接 SyntaxError。
    odom_amcl = amcl.get('odom_frame_id')
    if odom_amcl:
        if odom_amcl == fr['odom'] and gmapping.get('odom_frame') == fr['odom']:
            chk(OK, 'odom_frame 统一', fr['odom'])
        else:
            chk(FAIL, 'odom_frame 不一致',
                'amcl=%s, gmapping=%s, base=%s'
                % (odom_amcl, gmapping.get('odom_frame'), fr['odom']))

    # ==================================================================
    print(u'\n\033[36;1m── 3. 运动学与规划器匹配%s' % R)

    kine = rp['kinematics']
    if kine['type'] == 'omni' and not dwa.get('DWAPlannerROS', {}).get('holonomic_robot'):
        chk(FAIL, 'omni 底盘但 DWA holonomic_robot=false',
            '', '浪费横移能力；3m 窄场地会多做很多掉头')
    elif kine['type'] == 'omni':
        chk(OK, 'omni 底盘 + DWA 全向模式',
            'holonomic_robot=true，max_vel_y=%.2f'
            % dwa['DWAPlannerROS'].get('max_vel_y', 0))
    else:
        chk(OK, '差速底盘配置', 'type=%s' % kine['type'])

    if amcl['odom_model_type'] != kine['type']:
        chk(FAIL, 'AMCL 运动模型与底盘类型不符',
            'amcl=%s, base=%s' % (amcl['odom_model_type'], kine['type']),
            '用错模型的表现：侧向平移时定位持续漂移')
    else:
        chk(OK, 'AMCL 运动模型匹配', amcl['odom_model_type'])

    # ==================================================================
    print(u'\n\033[36;1m── 4. 赛题规则合规性%s' % R)

    dw = dwa.get('DWAPlannerROS', {})
    mvx = float(dw.get('max_vel_x', 0))
    mvt = float(dw.get('max_rot_vel', 0))
    if mvx >= 0.6:
        chk(OK, '最大线速度 ≥ 0.6 m/s（规则下限）', '%.2f m/s' % mvx)
    else:
        chk(FAIL, '最大线速度低于规则下限', '%.2f < 0.6 m/s' % mvx,
            '赛题要求最大速度 ≥0.6 m/s')

    if mvt >= 4.0:
        chk(OK, '最大转速 ≥ 4 rad/s（规则下限）', '%.2f rad/s' % mvt)
    else:
        chk(FAIL, '最大转速低于规则下限', '%.2f < 4.0 rad/s' % mvt,
            '赛题要求最大转速 ≥4 rad/s')

    if gcost['global_costmap']['robot_base_frame'] == 'base_footprint':
        chk(OK, 'costmap 使用 base_footprint', '与赛题平台一致')

    # ==================================================================
    print(u'\n\033[36;1m── 5. EKF 配置合法性%s' % R)

    pnc = ekf.get('process_noise_covariance')
    if pnc is None:
        chk(FAIL, '缺少 process_noise_covariance', '', 'EKF 无法启动')
    elif len(pnc) != 225:
        chk(FAIL, 'process_noise_covariance 长度错误',
            '%d，应为 225（15×15）' % len(pnc),
            '矩阵必须是 15×15=225 个元素')
    else:
        chk(OK, 'process_noise_covariance 是 15×15', '225 个元素')

    iec = ekf.get('initial_estimate_covariance')
    if iec is not None and len(iec) != 225:
        chk(FAIL, 'initial_estimate_covariance 长度错误',
            '%d，应为 225' % len(iec))
    elif iec is not None:
        chk(OK, 'initial_estimate_covariance 是 15×15', '225 个元素')

    for key in ('odom0_config', 'imu0_config'):
        v = ekf.get(key)
        if v is None:
            chk(WARN, '缺少 %s' % key, '')
        elif len(v) != 15:
            chk(FAIL, '%s 长度错误' % key, '%d，应为 15' % len(v))
        else:
            chk(OK, '%s 是 15 位' % key, '%d 个 true' % sum(1 for x in v if x))

    if ekf.get('world_frame') == ekf.get('odom_frame'):
        chk(OK, 'EKF world_frame == odom_frame',
            '只发布 odom→base_footprint，不与 amcl 抢 map→odom')
    else:
        chk(FAIL, 'EKF world_frame 设置错误',
            'world=%s, odom=%s' % (ekf.get('world_frame'), ekf.get('odom_frame')),
            'world_frame 必须等于 odom_frame，否则会与 amcl 抢 map→odom 的 TF')

    # ==================================================================
    print(u'\n\033[36;1m── 6. 话题链路一致性%s' % R)

    scan_topic = costmap_common['obstacle_layer']['scan']['topic']
    if scan_topic == '/scan_filtered':
        chk(OK, 'costmap 订阅滤波后的扫描', scan_topic)
    else:
        chk(FAIL, 'costmap 订阅了未滤波的扫描', scan_topic,
            '必须用 /scan_filtered，否则机器人把自己当障碍')

    # 检查 launch 里是否 remap 了
    mapping_launch = os.path.join(pkg, 'launch', 'mapping.launch')
    if os.path.isfile(mapping_launch):
        txt = io.open(mapping_launch, encoding='utf-8').read()
        if 'scan_filtered' in txt and 'laser_filter.launch' in txt:
            chk(OK, 'mapping.launch 包含并订阅 laser_filter', '链路完整')
        else:
            chk(FAIL, 'mapping.launch 缺少激光滤波', '',
                'gmapping 会订阅 /scan，把自身扫进地图')

    nav_launch = os.path.join(pkg, 'launch', 'navigation.launch')
    if os.path.isfile(nav_launch):
        txt = io.open(nav_launch, encoding='utf-8').read()
        missing = []
        for need in ('map_server', 'amcl', 'move_base', 'scan_filtered'):
            if need not in txt:
                missing.append(need)
        if missing:
            chk(FAIL, 'navigation.launch 缺少组件', ', '.join(missing))
        else:
            chk(OK, 'navigation.launch 组件完整',
                'map_server + amcl + move_base + 滤波')

    # ==================================================================
    print(u'\n\033[36;1m── 7. launch 引用的文件是否存在%s' % R)

    ref_re = re.compile(r'\$\(find\s+carebot_nav_kit\)([^\s"\'\)]*)')
    launch_files = []
    for root, _dirs, files in os.walk(os.path.join(pkg, 'launch')):
        for f in files:
            if f.endswith('.launch'):
                launch_files.append(os.path.join(root, f))

    missing_files = []
    for lf in launch_files:
        txt = io.open(lf, encoding='utf-8').read()
        for m in ref_re.finditer(txt):
            rel = m.group(1)
            if not rel:
                continue
            # 去掉 launch 参数后缀之类
            rel = rel.split(':')[0]
            target = os.path.join(pkg, rel.lstrip('/'))
            if not os.path.exists(target):
                missing_files.append((os.path.basename(lf), rel))

    if missing_files:
        for lf, rel in missing_files:
            chk(FAIL, '%s 引用了不存在的文件' % lf, rel,
                '创建该文件，或修正路径')
    else:
        chk(OK, '所有 $(find carebot_nav_kit) 引用都存在',
            '%d 个 launch 文件检查完毕' % len(launch_files))

    # ==================================================================
    print(u'\n\033[36;1m── 8. 里程计源与话题%s' % R)

    src = base['odometry']['source']
    topics = base['odometry']['topics']
    if src in topics:
        chk(OK, '里程计源已声明', '%s -> %s' % (src, topics[src]))
    else:
        chk(FAIL, '里程计源未在 topics 中定义', src)

    if ekf.get('odom0') == 'odom':
        chk(OK, 'EKF odom0 话题', ekf['odom0'])
    if ekf.get('imu0') == 'imu':
        chk(OK, 'EKF imu0 话题', ekf['imu0'])

    # ==================================================================
    # 汇总
    # ==================================================================
    n_ok = sum(1 for r in results if r[0] == OK)
    n_warn = sum(1 for r in results if r[0] == WARN)
    n_fail = sum(1 for r in results if r[0] == FAIL)

    print(u'\n' + u'=' * 74)
    color = C[FAIL] if n_fail else (C[WARN] if n_warn else C[OK])
    print(u'%s  结果：通过 %d 项，警告 %d 项，失败 %d 项  %s'
          % (color, n_ok, n_warn, n_fail, R))
    print(u'=' * 74)

    if n_fail:
        print(u'\n\033[31;1m存在配置错误，上车前必须修正。\033[0m')
    elif n_warn:
        print(u'\n\033[33;1m没有错误，但有警告项，建议确认。\033[0m')
    else:
        print(u'\n\033[32;1m配置自洽，可以打包上车。\033[0m')

    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())

