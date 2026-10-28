#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
utf8_stdio.py — 跨 Python 版本的中文输出支持

解决的问题（实测踩坑）
----------------------
脚本里打印中文（启动提示、状态播报、错误说明）时，在**没有 UTF-8 locale
的环境**下会崩：

    UnicodeEncodeError: 'ascii' codec can't encode characters
    in position 18-20: ordinal not in range(128)

为什么用 sys.stdout.reconfigure() 修不好
---------------------------------------
`TextIOWrapper.reconfigure()` 是 **Python 3.7 才加入**的。
目标环境里 Ubuntu 18.04 自带的是 **Python 3.6.9**，没有这个方法：

    >>> sys.stdout.reconfigure
    AttributeError: '_io.TextIOWrapper' object has no attribute 'reconfigure'

于是常见的
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
这种"兼容写法"会**静默跳过**编码设置 —— 本地用 Python 3.11 测一切正常，
一到目标机器就崩。这正是本项目实际发生过的 bug。

正确做法
--------
用 `codecs.getwriter` 包一层，这个 API 从 Python 2.6 就有：

    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer)

另外 `rosrun` 执行时环境里可能完全没有 LANG，所以还要在 Python 层面兜底，
不能依赖系统 locale。

用法
----
    from utf8_stdio import setup_stdout
    setup_stdout()

或者直接用提供的 print 封装：

    from utf8_stdio import uprint
    uprint(u'中文没问题')
"""

# ⚠️ 这一行**不能删**。
#
# `uprint` / `eprint` 里用了 `print(*args, **kwargs)`。
# 在 Python 2 下，`print` 是**语句**而不是函数，所以
#     print(*args, **kwargs)
# 会被解析成 `print (tuple)`，而 `*args` 出现在元组里是**语法错误** ——
# 整个文件 import 时就会 SyntaxError。
#
#    Python 2.7:
#        >>> print(*[1,2], **{})
#        SyntaxError: invalid syntax
#
# 加了 `print_function` 之后，`print` 在 Py2 下也是函数，
# `print(*args, **kwargs)` 就合法了。
#
# ⚠️ 而且这一行**必须放在所有其它 import 之前**（PEP 236）——
#    放到后面 Python 会直接报 SyntaxError。
#
# 这个 future 特性从 **Python 2.6** 就有了，Py3 下是空操作，
# 所以加它对两个版本都安全。
from __future__ import print_function

import codecs
import sys

__all__ = ['setup_stdout', 'setup_stderr', 'setup_all', 'uprint', 'eprint',
           'to_text', 'to_bytes']

_INSTALLED = False


def to_text(v):
    """把任意值转成 **unicode**，且不因非 ASCII 而崩。

    ## 为什么需要这个

    这是本项目**反复踩了三次**的同一类 bug：

        Python 2 里，把 **字节串（str）** 插进 **unicode 格式串** 时，
        会先按 **ASCII** 解码那个字节串：

            >>> u'%s' % '\\xe4\\xb8\\xad'      # 一个 UTF-8 的"中"字
            UnicodeDecodeError: 'ascii' codec can't decode byte 0xe4

        也就是说 **出错的是"混合编码"，不是"中文"**。

    实测踩过的三个地方：

      1. `u'...%r' % exc`
         `%r` 产出的是 **str**，异常消息含中文就崩；
         **而且这行常常在 except 里**，一崩就把真异常也盖掉了。

      2. 颜色常量是字节串：
             C = {OK: '\\033[32;1m'}
             print(u'%s[%s]%s' % (C[level], level, R))
         颜色串是 ASCII，看着没问题；但 `%-44s` 那一位如果拿到
         **非 ASCII 字节串**（比如中文名），整个格式化就崩。

      3. 测试代码里 `print(u'\\033[36;1m...%s' % R)` —— 同一个病。

    ## 用法

        from utf8_stdio import to_text
        print(u'  %s %s' % (to_text(color), to_text(name)))

    ⚠️ 注意：**只有 Python 2 需要这么小心**。Py3 里 str 就是 unicode。
       但目标机（ROS Kinetic / Melodic）都是 Py2.7，所以必须这么写。
    """
    if v is None:
        return u''
    if isinstance(v, bytes):
        # Py3 下 bytes 也是 bytes；Py2 下 str 走这里
        try:
            return v.decode('utf-8')
        except Exception:
            try:
                return v.decode('utf-8', 'replace')
            except Exception:
                return repr(v)
    try:
        if isinstance(v, unicode):      # noqa: F821  (Py2)
            return v
    except NameError:
        pass
    try:
        return u'%s' % (v,)
    except Exception:
        return repr(v)


def to_bytes(v, encoding='utf-8'):
    """把任意值转成 UTF-8 字节串（写文件、发 ROS 消息时用）。"""
    if v is None:
        return b''
    if isinstance(v, bytes):
        return v
    try:
        if isinstance(v, unicode):      # noqa: F821  (Py2)
            return v.encode(encoding, 'replace')
    except NameError:
        pass
    return (u'%s' % (v,)).encode(encoding, 'replace')


def _wrap(stream, encoding='utf-8'):
    """把一个文本流重新包装成指定编码。

    ⚠️ 必须容忍**字节串**输入 —— Python 2 里 traceback.print_exc() 和
       logging.StreamHandler 都往流里写字节串。标准的 codecs.StreamWriter
       只接受 unicode，收到字节串会先 decode('ascii')，遇到非 ASCII 就崩：
           UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5
       后果是"打印异常时崩掉"，真正的错误再也看不到。
    """
    import codecs as _codecs

    class _Utf8Writer(_codecs.StreamWriter):
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

    if stream is None:
        return None

    enc = (getattr(stream, 'encoding', None) or '').lower().replace('-', '')
    if enc in ('utf8', 'utf_8'):
        return stream

    # Python 3 用 .buffer；Python 2 没有，直接用流本身
    buf = getattr(stream, 'buffer', None)
    if buf is None:
        buf = stream
    try:
        return _Utf8Writer(buf, errors='backslashreplace')
    except Exception:
        pass

    reconf = getattr(stream, 'reconfigure', None)
    if callable(reconf):
        try:
            reconf(encoding='utf-8', errors='backslashreplace')
            return stream
        except Exception:
            pass
    return stream

def setup_stdout(encoding='utf-8'):
    """让 sys.stdout 能输出任意 Unicode（中文等）。"""
    global _INSTALLED
    sys.stdout = _wrap(sys.stdout, encoding)
    _INSTALLED = True
    return sys.stdout


def setup_stderr(encoding='utf-8'):
    """让 sys.stderr 能输出任意 Unicode。"""
    sys.stderr = _wrap(sys.stderr, encoding)
    return sys.stderr


def setup_all(encoding='utf-8'):
    """同时处理 stdout 和 stderr —— 节点里调这个就够了。"""
    setup_stdout(encoding)
    setup_stderr(encoding)
    return _INSTALLED


def uprint(*args, **kwargs):
    """安全打印：先确保 stdin/stdout/stderr 能编码，再 print。"""
    if not _INSTALLED:
        setup_all()
    kwargs.setdefault('file', sys.stdout)
    return print(*args, **kwargs)


def eprint(*args, **kwargs):
    """安全打印到 stderr。"""
    if not _INSTALLED:
        setup_all()
    kwargs.setdefault('file', sys.stderr)
    return print(*args, **kwargs)


if __name__ == '__main__':
    # 自测：模拟无 UTF-8 locale 的情况
    setup_all()
    print(u'[utf8_stdio] 中文输出测试：启动器就绪、状态机运行中')
    print(u'[utf8_stdio] stdout.encoding = %s' % getattr(sys.stdout, 'encoding', '?'))
    print(u'[utf8_stdio] stderr.encoding = %s' % getattr(sys.stderr, 'encoding', '?'))
    print(u'[utf8_stdio] 特殊字符：⚠️ ✔ ✘ → ≥ ² °C')
