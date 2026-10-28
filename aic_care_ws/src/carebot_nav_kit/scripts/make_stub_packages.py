#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
make_stub_packages.py — 为缺失的依赖包生成桩包（stub）

为什么需要这个
--------------
`carebot_nav_kit` 的设计目标是"不硬依赖组委会的包"，让整套东西能在不同配置的
机器上跑（虚拟机、实车、别人借的机器）。

但**ROS 1 的 roslaunch 机制上做不到优雅降级**，实测确认：

    <group if="$(eval find('base_control') is not None)">
      <include file="$(find base_control)/launch/stm32.launch" />
    </group>

即使条件为假、整个 group 被跳过，roslaunch 仍然会**先求值子元素里的
$(find base_control)**，然后才判断条件 —— 于是直接抛
    Resource not found: base_control

对照实验（VM 上实测）：
    driver=stm32  -> 失败（进了 if 分支）
    driver=none   -> OK  （整个 group 不进，$(find) 不求值）
                 -> 说明"包不存在"本身无法被 if 挡住

结论：想让 launch 在缺包时还能启动，就必须让 `rospack find <包>` 能成功。

解法
----
本脚本扫描工作空间里所有 launch 文件，找出被 `$(find X)` 引用但实际不存在的包，
为每个缺失包生成一个**最小的桩包**：
    <工作空间>/src/_stubs/<包名>/
        package.xml          合法的 catkin 包描述
        launch/stm32.launch  等：被引用到的 launch 文件，内容只打印一条提示

这样：
  · `rospack find X` 成功 → roslaunch 解析通过
  · 真正 include 那个 launch 时，只输出"这个包是桩，请安装真包"
  · 不需要 catkin_make（桩包没有可编译内容，roslaunch 靠包路径就能找到）

用法
----
    # 在工作空间里执行
    python3 make_stub_packages.py                    # 扫描并生成
    python3 make_stub_packages.py --dry-run          # 只看缺什么，不生成
    python3 make_stub_packages.py --clean            # 删除已生成的桩包
    python3 make_stub_packages.py --ws ~/catkin_ws   # 指定工作空间
"""

import argparse
import io
import os
import re
import subprocess
import sys


def safe_makedirs(path):
    """跨 Python 版本的"目录已存在也不报错"建目录。

    ⚠️ 为什么不能用 `os.makedirs(path, exist_ok=True)`：

        `exist_ok` 是 **Python 3.2 才加的关键字参数**。
        在 Python 2.7 下（本项目的目标环境之一）会直接抛：

            TypeError: makedirs() got an unexpected keyword argument
                       'exist_ok'

    而这个文件本来就要兼容 Python 2.7 / 3.6 / 3.11
    （见下面 setup_stdout 的说明），所以得用两个版本都有的写法。

    项目里其他地方（mission_manager.py 等）也是用
    `if not os.path.isdir(...)` 这个模式，保持一致。
    """
    if not path:
        return
    if not os.path.isdir(path):
        try:
            os.makedirs(path)
        except OSError:
            # 并发或权限边缘情况：再确认一次，真的是目录就当成功
            if not os.path.isdir(path):
                raise


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
    unicode  # noqa: F821
except NameError:
    unicode = str  # noqa: A001

# 这些包不需要桩（它们本来就应该存在，缺了说明 ROS 装坏了）
IGNORE = set([
    'carebot_eldercare', 'carebot_nav_kit', 'carebot_qr', 'carebot_iot',
    'carebot_voice', 'carebot_face',
])

PKG_REF = re.compile(r'\$\(find\s+([A-Za-z0-9_\-]+)\)')
# 提取 file="$(find X)/相对路径" 形式，用于知道要生成哪些 launch 文件
FILE_REF = re.compile(r'file="\$\(find\s+([A-Za-z0-9_\-]+)\)([^"]*)"')


def rospack_find(pkg):
    """包是否存在及路径。"""
    try:
        out = subprocess.Popen(['rospack', 'find', pkg],
                               stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL).communicate()[0]
        out = out.decode('utf-8', 'replace').strip()
        return out if out and 'not found' not in out.lower() else None
    except Exception:
        return None


def scan_launches(ws_src):
    """扫描所有 launch 文件，返回 {包名: set(被引用的相对路径)}。"""
    refs = {}
    for root, _dirs, files in os.walk(ws_src):
        for f in files:
            if not f.endswith('.launch'):
                continue
            path = os.path.join(root, f)
            try:
                with io.open(path, encoding='utf-8', errors='replace') as fp:
                    content = fp.read()
            except Exception:
                continue
            # 去掉 XML 注释，避免把注释里的示例当成真引用
            content = re.sub(r'<!--.*?-->', '', content, flags=re.S)

            for m in PKG_REF.finditer(content):
                pkg = m.group(1)
                refs.setdefault(pkg, set())
            for m in FILE_REF.finditer(content):
                pkg, rel = m.group(1), m.group(2)
                if rel.startswith('/'):
                    refs.setdefault(pkg, set()).add(rel.lstrip('/'))
    return refs


def make_package_xml(pkg):
    return u'''<?xml version="1.0"?>
<!--
  自动生成的**桩包** —— 不是真实功能包！

  生成原因：工作空间里的 launch 文件引用了 $(find %s)，
            但该包不在 ROS 路径中，导致 roslaunch 直接抛
            "Resource not found: %s" 而无法启动。

  这个桩包的作用只是让 rospack find %s 成功，
  从而让 launch 文件能被解析。它不含任何真实功能。

  如何换成真包：
    把组委会源码里的 %s 放到 src/ 下，然后删除本目录：
        rm -rf src/_stubs/%s
        catkin_make

  由 make_stub_packages.py 生成，可安全删除。
-->
<package format="2">
  <name>%s</name>
  <version>0.0.0</version>
  <description>STUB package (auto-generated) - replace with the real package</description>
  <maintainer email="stub@example.com">stub</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
''' % (pkg, pkg, pkg, pkg, pkg, pkg)


def make_stub_launch(pkg, rel_path, purpose_hint=u''):
    """生成一个只打印提示的 launch 文件，占住被引用的路径。"""
    return u'''<launch>
  <!--
    桩 launch 文件 —— 由 make_stub_packages.py 自动生成

    真实的 $(find %s)%s 不存在。
    本文件只打印一条提示，不做任何实际工作。

    ⚠️ 这意味着依赖这个包的子系统**不会真正启动**。
       如果你需要它，请安装真实的 %s 包。
  -->
  <node pkg="carebot_nav_kit" type="missing_pkg_notice.py"
        name="stub_notice_%s"
        args="%s 桩包：真实包未安装" output="screen"
        required="false" />
</launch>
''' % (pkg, rel_path, pkg,
       re.sub(r'[^A-Za-z0-9_]', '_', pkg),
       pkg)


def main():
    ap = argparse.ArgumentParser(description=u'为缺失依赖生成桩包')
    ap.add_argument('--ws', default=os.path.expanduser('~/catkin_ws'),
                    help=u'catkin 工作空间路径')
    ap.add_argument('--dry-run', action='store_true', help=u'只报告，不生成')
    ap.add_argument('--clean', action='store_true', help=u'删除所有桩包')
    args = ap.parse_args()

    ws = os.path.abspath(os.path.expanduser(args.ws))
    src = os.path.join(ws, 'src')
    stub_root = os.path.join(src, '_stubs')

    if not os.path.isdir(src):
        print(u'找不到工作空间 src：%s' % src)
        return 1

    # ---------------------------------------------------------------- clean
    if args.clean:
        if os.path.isdir(stub_root):
            import shutil
            shutil.rmtree(stub_root)
            print(u'已删除桩包目录：%s' % stub_root)
        else:
            print(u'没有桩包目录')
        return 0

    print(u'\n扫描工作空间：%s' % src)
    refs = scan_launches(src)
    print(u'  发现 %d 个不同的包引用' % len(refs))

    # 找出缺失的
    missing = {}
    present = []
    for pkg in sorted(refs):
        if pkg in IGNORE:
            continue
        path = rospack_find(pkg)
        if path:
            present.append(pkg)
        else:
            missing[pkg] = refs[pkg]

    print(u'  已存在 : %d 个' % len(present))
    print(u'  缺失   : %d 个' % len(missing))

    if not missing:
        print(u'\n✔ 所有被引用的包都存在，不需要桩包')
        return 0

    print(u'\n缺失的包及其被引用的 launch 文件：')
    for pkg in sorted(missing):
        rels = sorted(missing[pkg])
        print(u'  %-24s (%d 个引用)' % (pkg, len(rels)))
        for r in rels:
            print(u'      %s' % r)

    if args.dry_run:
        print(u'\n[--dry-run] 未生成任何文件')
        return 0

    # ---------------------------------------------------------------- 生成
    print(u'\n生成桩包到：%s' % stub_root)
    made = 0
    for pkg in sorted(missing):
        pkg_dir = os.path.join(stub_root, pkg)
        safe_makedirs(pkg_dir)

        # package.xml
        pxml = os.path.join(pkg_dir, 'package.xml')
        with io.open(pxml, 'w', encoding='utf-8') as fp:
            fp.write(make_package_xml(pkg))

        # CMakeLists.txt（catkin 需要，哪怕没内容）
        cml = os.path.join(pkg_dir, 'CMakeLists.txt')
        if not os.path.isfile(cml):
            with io.open(cml, 'w', encoding='utf-8') as fp:
                fp.write(u'''cmake_minimum_required(VERSION 3.0.2)
project(%s)
find_package(catkin REQUIRED)
catkin_package()
''' % pkg)

        # 被引用到的 launch 文件
        for rel in sorted(missing[pkg]):
            target = os.path.join(pkg_dir, rel)
            safe_makedirs(os.path.dirname(target))
            with io.open(target, 'w', encoding='utf-8') as fp:
                fp.write(make_stub_launch(pkg, '/' + rel))

        # 至少有一个 launch 目录，方便人工看
        safe_makedirs(os.path.join(pkg_dir, 'launch'))
        made += 1
        print(u'  ✔ %s' % pkg)

    print(u'\n共生成 %d 个桩包' % made)
    print(u'''
下一步：
    cd %s
    catkin_make          # 让 catkin 认识这些桩包
    source devel/setup.bash

    # 验证
    rospack find base_control        # 应该能返回路径

⚠️ 桩包只是让 launch 能被解析，**不含真实功能**。
   需要真功能时，把真实包装到 src/ 下并删除对应桩包：
       rm -rf src/_stubs/<包名>
       catkin_make
''' % ws)

    return 0


if __name__ == '__main__':
    setup_stdout()
    sys.exit(main())
