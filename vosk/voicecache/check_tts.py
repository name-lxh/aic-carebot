#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""check_tts.py —— 静态核对：任务可能说的每句话，能否全部由缓存片段拼出来。

不起 ROS、不动车；只读 manifest.json + cache/，跑和 tts_play 一样的
归一化与最长匹配切词，把"拼不出来的片段"列出来。
用法： python2 check_tts.py [phrases.txt]
"""
from __future__ import print_function

import codecs
import hashlib
import io
import json
import os
import re
import sys

try:
    if not getattr(sys.stdout, 'encoding', None) or \
            'utf' not in sys.stdout.encoding.lower():
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout)
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, 'cache')
MANIFEST = os.path.join(HERE, 'manifest.json')
DIGITS = u'零一二三四五六七八九'
UNIT_REPLACE = [(u'°C', u'摄氏度'), (u'℃', u'摄氏度'), (u'lx', u'勒克斯'),
                (u'LX', u'勒克斯'), (u'%', u'百分之')]


def integer_cn(n):
    if n < 10:
        return DIGITS[n]
    if n < 20:
        return u'十' + (DIGITS[n - 10] if n > 10 else u'')
    out, zero = u'', False
    for value, name in ((1000, u'千'), (100, u'百'), (10, u'十'), (1, u'')):
        digit, n = divmod(n, value)
        if digit:
            if zero:
                out += u'零'
                zero = False
            out += DIGITS[digit] + name
        elif out:
            zero = True
    return out


def number_cn(text):
    sign = u'负' if text.startswith('-') else u''
    whole, dot, frac = text.lstrip('-').partition('.')
    out = sign + integer_cn(int(whole or '0'))
    if dot:
        out += u'点' + u''.join(DIGITS[int(c)] for c in frac)
    return out


def normalise(text):
    for src, dst in UNIT_REPLACE:
        text = text.replace(src, dst)
    text = re.sub(r'-?\d+(?:\.\d+)?', lambda m: number_cn(m.group(0)), text)
    return re.sub(u'[\\s，。！？、：；,\.!?:;（）()【】\\[\\]"“”\'‘’]+', u'', text)


def main():
    manifest = json.load(io.open(MANIFEST, encoding='utf-8'))
    keys = sorted({normalise(t) for t in manifest}, key=len, reverse=True)
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'phrases.txt')
    lines = [l.strip() for l in io.open(path, encoding='utf-8') if l.strip()]

    bad = 0
    for text in lines:
        norm = normalise(text)
        i, miss = 0, u''
        while i < len(norm):
            hit = next((k for k in keys if k and norm.startswith(k, i)), None)
            if hit:
                if miss:
                    print(u'  ✗ %s  → 缺片段「%s」' % (text, miss))
                    bad += 1
                    miss = u''
                i += len(hit)
            else:
                miss += norm[i]
                i += 1
        if miss:
            print(u'  ✗ %s  → 缺片段「%s」' % (text, miss))
            bad += 1
    print(u'检查 %d 句：%s' % (len(lines), u'全部可拼 ✓' if bad == 0 else u'%d 处缺口' % bad))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
