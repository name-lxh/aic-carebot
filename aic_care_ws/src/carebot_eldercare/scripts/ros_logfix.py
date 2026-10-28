#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ros_logfix.py — 修好 ROS 1 在 Python 2 下打印中文日志崩溃的问题

## 问题

ROS Melodic / Kinetic 都跑 Python 2.7。Py2 的 `str(unicode)` 默认按 **ASCII**
编码，而 ROS 的日志处理器就是这么转的：

    rosgraph/roslogging.py:249
        msg = msg.replace('${message}', str(record_message))

于是任何一句

    rospy.loginfo(u'已加载 6 个航点：%s' % names)

都会抛

    UnicodeEncodeError: 'ascii' codec can't encode characters

## 为什么这个坑特别毒（本项目踩了 5 次以上）

它**不是"日志没打出来"这么简单** —— 异常会沿调用栈往上冒，
把**正常逻辑伪装成失败**：

    try:
        data = yaml.safe_load(...)              # 成功
        self.waypoints = {...}                   # 成功
        loginfo(u'已加载 %d 个航点' % n)          # ← 这里抛
    except Exception as exc:
        logerr(u'航点文件解析失败：%s' % exc)      # ← 报的是假错

日志里只剩「解析失败」，exc 说的是 `'ascii' codec can't decode...`，
**看起来像文件编码问题**，实际文件是好好的 UTF-8。
我为此查了好几轮 —— 直到手动跑一次，看到
「失败」的下一行就是「已加载 6 个航点」才发现真相。

同一个坑还出现过：
  · `sim_stubs.py` —— 第一句中文日志让 `__init__` 中断，
    **四个 ROS 服务一个都没注册上**，表现为"服务不存在"
  · `mission_manager.py` —— 同样的补丁复制了一份

## 用法

    from ros_logfix import fix_ros_logging, setup_stdout
    setup_stdout()          # 让 print 中文不崩
    fix_ros_logging()       # 让 rospy.log* 中文不崩（在 init_node 前后都行）
    rospy.init_node('xxx')

⚠️ 本文件要放在**与节点同一个目录**（`nodes/` 或 `scripts/`）——
   ROS 1 的 launch 会把节点从源码目录直接执行，
   同目录的模块能被 import 到。

## 它做了什么

在 log record 进入 handler **之前**，把 unicode 消息与参数转成 UTF-8 字节串。
这样 roslogging 里那句 `str(record_message)` 就是"字节串转字节串"，
是无操作，不会再触发 ASCII 编码。

⚠️ 这个补丁**不是万能的**：ROS 里有两处会做 `str(unicode)`，
   两处叠加时仍可能出现
       UnicodeDecodeError: 'ascii' codec can't decode byte ...
   （注意是 **Decode**Error —— 字节串又被当文本解了一次）。

   所以**新写的日志请尽量用 ASCII**，中文说明放注释里。
   补丁是兜底，不是许可证。
"""

from __future__ import absolute_import


def fix_ros_logging():
    """打上 UTF-8 日志补丁。可重复调用，只生效一次。"""
    import logging as _logging

    if getattr(_logging.Logger, '_ros_utf8_patched', False):
        return

    try:
        _text_types = (unicode,)            # noqa: F821  (Py2)
    except NameError:
        _text_types = (str,)                # Py3

    _orig = _logging.Logger.makeRecord

    def _utf8_makeRecord(self, name, level, fn, lno, msg, args, exc_info,
                         func=None, extra=None, sinfo=None):
        if isinstance(msg, _text_types):
            try:
                msg = msg.encode('utf-8')
            except Exception:
                pass
        if args:
            try:
                args = tuple(
                    a.encode('utf-8') if isinstance(a, _text_types)
                    else a for a in args)
            except Exception:
                pass
        try:
            return _orig(self, name, level, fn, lno, msg, args,
                         exc_info, func, extra, sinfo)
        except TypeError:
            # Python 2.7 的签名没有 sinfo
            return _orig(self, name, level, fn, lno, msg, args,
                         exc_info, func, extra)

    _logging.Logger.makeRecord = _utf8_makeRecord
    _logging.Logger._ros_utf8_patched = True


def setup_stdout():
    """让 stdout/stderr 能输出中文（Py2 / Py3 都能用）。

    ## 为什么委托给 `utf8_stdio`

    原来这里自己写了一份 `codecs.getwriter` 包装。能用，但有**两个**问题：

      1. **不容忍字节串输入。**
         Py2 里 `traceback.print_exc()` 和 `logging.StreamHandler`
         都往流里写**字节串**。`codecs.getwriter` 返回的标准 StreamWriter
         只接受 unicode，收到字节串会先 `decode('ascii')` → 崩：
             UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5
         后果是"**为了看清异常而打印异常时崩掉**"，真正的错误彻底看不到。
         （单元测试 `test_stream_writer_tolerates_bytes` 抓的就是这个。）

      2. **同一份逻辑存在多处实现。**
         本项目一度有 5 个文件各自内联编码处理，其中还出现过
         "对已被包过的流再包一层 → UnicodeDecodeError" 的事故。
         **编码处理只应有一份实现。**

    `utf8_stdio.setup_all()` 两个问题都解决了：
    它用自定义 `_Utf8Writer`（继承 `codecs.StreamWriter` 但覆写 `write`
    以容忍 bytes），且是项目里唯一的一份实现。

    ⚠️ `nodes/` 与 `scripts/` 是两个目录，模块不一定能互相 import。
       所以按"同目录 → 加上级 scripts/ → 退化成最小实现"三级回退。
    """
    import os
    import sys

    # 1) 同目录（agent 常常把 ros_logfix 和 utf8_stdio 放一起）
    try:
        from utf8_stdio import setup_all
        setup_all()
        return
    except ImportError:
        pass

    # 2) 同包的 scripts/ 目录（nodes/ 旁边的 scripts/）
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in (os.path.join(here, '..', 'scripts'),):
            cand = os.path.abspath(cand)
            if os.path.isdir(cand) and cand not in sys.path:
                sys.path.insert(0, cand)
        from utf8_stdio import setup_all
        setup_all()
        return
    except ImportError:
        pass

    # 3) 退化：最小可用实现（**仍然容忍字节串**，否则会掩盖真异常）
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
