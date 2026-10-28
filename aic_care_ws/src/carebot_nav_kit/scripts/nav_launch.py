#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
nav_launch.py — 硬件配置驱动的启动器

解决的问题
----------
原来有 `config/hardware/*.yaml` 这些硬件 profile，写了"换硬件只改 profile"，
但**实际上没有任何 launch 会读它们** —— 它们只是文档，不是配置源。
使用者改 profile 发现没效果，会浪费时间排查。

本脚本把 profile 变成**真正的配置源**：

    读 profile YAML  →  取出各子系统的驱动名  →  转换成 roslaunch 参数  →  启动

为什么不用纯 launch 实现
------------------------
ROS 1 的 launch XML 没有"读 YAML 再决定 include 谁"的能力：
  · `$(eval ...)` 不能读文件
  · `$(arg ...)` 只能取命令行传进来的值
所以要在 XML 里做 profile 驱动，只能把整张映射表硬编码进每个 launch，
而且 `$(find 缺失包)` 还会在条件判断之前就被求值（见 base.launch 的说明）。

用脚本做这一层，逻辑集中、可测试、可给出清楚的错误提示。

用法
----
    # 用默认 profile（lab_carebot）
    python3 nav_launch.py bringup

    # 指定 profile
    python3 nav_launch.py mapping --profile alt_arduino
    python3 nav_launch.py navigation --profile pi_rplidar_a1m8

    # 覆盖单个驱动
    python3 nav_launch.py bringup --laser rplidar

    # 只看会启动什么，不实际启动
    python3 nav_launch.py bringup --dry-run

    # 列出所有可用 profile
    python3 nav_launch.py --list-profiles
"""

import argparse
import io
import os
import subprocess
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

try:
    import yaml
except ImportError:
    sys.stderr.write('需要 pyyaml：sudo apt install python3-yaml\n')
    sys.exit(2)

try:
    unicode  # noqa: F821
except NameError:
    unicode = str  # noqa: A001


# ---------------------------------------------------------------------------
# profile 里的 launch_pkg 名字 → launch 文件里 --arg 的 driver 取值
# ---------------------------------------------------------------------------
#  base.launch 的 driver 取值：stm32 | arduino | none
#  laser.launch 的 driver 取值：lslidar_m10 | ls01d | rplidar | none
#  imu.launch 的 driver 取值：fdilink | art_imu | none
PKG_TO_DRIVER = {
    # 底盘
    'base_control':       ('base', 'stm32'),
    'ros_arduino_python': ('base', 'arduino'),
    # 雷达
    'lslidar_driver':     ('laser', 'lslidar_m10'),
    'lsm10_v2':           ('laser', 'lslidar_m10'),
    'ls01d_lidar':        ('laser', 'ls01d'),
    'rplidar_ros':        ('laser', 'rplidar'),
    # IMU
    'fdilink_ahrs':       ('imu', 'fdilink'),
    'art_imu_01':         ('imu', 'art_imu'),
}

SUBSYS_LABEL = {'base': u'底盘', 'laser': u'激光雷达', 'imu': u'IMU'}


def find_pkg_dir():
    """定位 carebot_nav_kit 包目录。"""
    try:
        out = subprocess.Popen(['rospack', 'find', 'carebot_nav_kit'],
                               stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL).communicate()[0]
        p = out.decode('utf-8', 'replace').strip()
        if p and os.path.isdir(p):
            return p
    except Exception:
        pass
    # 退化：按脚本自身位置推
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(here)


def load_profile(pkg_dir, name):
    path = os.path.join(pkg_dir, 'config', 'hardware', name + '.yaml')
    if not os.path.isfile(path):
        return None, path
    with open(path, "rb") as fp:
        return yaml.safe_load(fp.read()) or {}, path


def list_profiles(pkg_dir):
    d = os.path.join(pkg_dir, 'config', 'hardware')
    if not os.path.isdir(d):
        return []
    out = []
    for f in sorted(os.listdir(d)):
        if f.endswith('.yaml'):
            out.append(f[:-5])
    return out


def profile_to_drivers(profile):
    """把 profile 的 subsystems 转成 {base/laser/imu: driver}。

    返回 (drivers, others)：
      drivers —— 导航启动器能处理的驱动选择
      others  —— 已启用但**不归导航启动器管**的子系统，例如深度相机
                 （相机的启动由 carebot_face / carebot_qr 负责）。
                 这些要显式报告给用户，不能静默忽略。
    """
    drivers = {}
    others = []
    subs = profile.get('subsystems') or {}

    for key in ('base', 'laser', 'imu'):
        s = subs.get(key) or {}
        if not s.get('enable', False):
            drivers[key] = 'none'
            continue
        pkg = s.get('launch_pkg') or ''
        if not pkg:
            drivers[key] = 'none'
            continue
        mapped = PKG_TO_DRIVER.get(pkg)
        if mapped is None:
            sys.stderr.write(
                u'⚠️  profile 里的包 %s 没有对应的 driver 取值映射。\n'
                u'    已知映射：%s\n'
                u'    请在 nav_launch.py 的 PKG_TO_DRIVER 里补充。\n'
                % (pkg, ', '.join(sorted(PKG_TO_DRIVER))))
            drivers[key] = 'none'
        else:
            drivers[mapped[0]] = mapped[1]

    # 收集"启用但不归本启动器管"的子系统
    for key, s in sorted(subs.items()):
        if key in ('base', 'laser', 'imu'):
            continue
        if not s.get('enable'):
            continue
        others.append({
            'key': key,
            'pkg': s.get('launch_pkg') or '',
            'launch_file': s.get('launch_file') or '',
            'desc': s.get('desc') or key,
        })

    return drivers, others


def main():
    ap = argparse.ArgumentParser(
        description=u'硬件配置驱动的导航启动器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('target', nargs='?', default='bringup',
                    choices=['bringup', 'mapping', 'navigation'],
                    help=u'要启动的 launch（默认 bringup）')
    ap.add_argument('--profile', default='lab_carebot',
                    help=u'硬件 profile 名（config/hardware/ 下，不含 .yaml）')
    ap.add_argument('--base', help=u'覆盖底盘驱动：stm32|arduino|none')
    ap.add_argument('--laser', help=u'覆盖雷达驱动：lslidar_m10|ls01d|rplidar|none')
    ap.add_argument('--imu', help=u'覆盖 IMU 驱动：fdilink|art_imu|none')
    ap.add_argument('--list-profiles', action='store_true',
                    help=u'列出所有可用 profile')
    ap.add_argument('--dry-run', action='store_true',
                    help=u'只打印将要执行的命令')
    ap.add_argument('extra', nargs='*',
                    help=u'额外的 roslaunch 参数，如 map_file:=...')
    args = ap.parse_args()

    pkg_dir = find_pkg_dir()

    # ---------------------------------------------------------------- 列 profile
    if args.list_profiles:
        print(u'可用的硬件 profile（%s）：' % os.path.join(pkg_dir, 'config', 'hardware'))
        for name in list_profiles(pkg_dir):
            prof, _ = load_profile(pkg_dir, name)
            desc = (prof or {}).get('profile_desc', u'')
            print(u'  %-20s %s' % (name, desc))
        return 0

    # ---------------------------------------------------------------- 读 profile
    profile, path = load_profile(pkg_dir, args.profile)
    if profile is None:
        sys.stderr.write(u'找不到 profile：%s\n' % path)
        sys.stderr.write(u'可用：%s\n' % ', '.join(list_profiles(pkg_dir)))
        return 1

    drivers, others = profile_to_drivers(profile)

    # 命令行覆盖
    for key in ('base', 'laser', 'imu'):
        v = getattr(args, key)
        if v:
            drivers[key] = v

    # ---------------------------------------------------------------- 打印计划
    overridden = any([args.base, args.laser, args.imu])
    print(u'\n' + u'=' * 66)
    print(u'  carebot_nav_kit 启动器')
    print(u'=' * 66)
    print(u'  目标     : %s.launch' % args.target)
    print(u'  profile  : %s' % args.profile)
    print(u'  说明     : %s' % (profile.get('profile_desc') or u'(无)'))
    print()
    print(u'  驱动选择（%s）：' % (u'profile + 命令行覆盖' if overridden else u'全部来自 profile'))
    for key in ('base', 'laser', 'imu'):
        marker = u'  ← 命令行覆盖' if getattr(args, key) else u''
        print(u'    %-8s : %-14s%s' % (SUBSYS_LABEL[key], drivers[key], marker))
    print()
    print(u'  里程计融合 : %s' % (u'开启' if (profile.get('odometry') or {}).get('use_ekf') else u'关闭'))

    # 显式报告"启用了但导航启动器不管"的子系统，避免用户以为它们会被启动
    if others:
        print()
        print(u'  以下子系统已启用，但**不由本启动器启动**（需另行启动）：')
        for o in others:
            if o['pkg']:
                print(u'    %-8s : %s  (%s)' % (o['key'], o['desc'], o['pkg']))
            else:
                print(u'    %-8s : %s' % (o['key'], o['desc']))
        print(u'    → 相机由 carebot_face / carebot_qr 负责，不归导航包管')

    print(u'=' * 66)
    print()

    # 检查这些驱动对应的包在不在（用桩包也算在）
    print(u'  依赖包检查：')
    subs = profile.get('subsystems') or {}
    for key in ('base', 'laser', 'imu'):
        s = subs.get(key) or {}
        pkg = s.get('launch_pkg') or ''
        if not pkg or drivers.get(key) == 'none':
            continue
        try:
            out = subprocess.Popen(['rospack', 'find', pkg],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.DEVNULL).communicate()[0]
            p = out.decode('utf-8', 'replace').strip()
        except Exception:
            p = ''
        if p and os.path.isdir(p):
            is_stub = '_stubs' in p
            tag = u'桩包（无真实功能）' if is_stub else u'真实包'
            # ⚠️ `p` 来自 os.path.dirname()，Py2 下返回**字节串**；
            #    插进含中文的 unicode 格式串会崩。
            #    `pkg` 来自 yaml 的 launch_pkg（yaml 读字节串，
            #    也是字节串）。两者都要先转 unicode。
            print(u'    [有] %-22s %s  %s'
                  % (to_text(pkg), to_text(p), tag))
        else:
            print(u'    [缺] %-22s 需要生成桩包或安装真实包' % pkg)
    print()

    # ---------------------------------------------------------------- 组装命令
    cmd = ['roslaunch', 'carebot_nav_kit', args.target + '.launch',
           'base_driver:=' + drivers['base'],
           'laser_driver:=' + drivers['laser'],
           'imu_driver:=' + drivers['imu']]
    cmd.extend(args.extra)

    print(u'  执行命令：')
    print(u'    ' + u' '.join(cmd))
    print()

    if args.dry_run:
        print(u'  [--dry-run] 未实际启动')
        return 0

    # ---------------------------------------------------------------- 启动
    try:
        return subprocess.call(cmd)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        sys.stderr.write(u'启动失败：%s\n' % repr(exc))
        return 1


if __name__ == '__main__':
    setup_stdout()
    sys.exit(main())
