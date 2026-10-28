# -*- coding: utf-8 -*-
"""检查我们的固定播报句有多少已经在既有离线语音缓存里。

既有缓存（92 条 WAV，按文本 MD5 命名）放在 _tools/voicecache/cache。
这个脚本：从 m36 + voice_dialog 抽出所有**静态**播报文本，算 MD5，看有没有对应 WAV。
"""
from __future__ import print_function

import hashlib
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CACHE = os.path.join(ROOT, '_tools', 'voicecache', 'cache')
SRC = [os.path.join(ROOT, '_tools', 'real_test', 'm36_mission_v5.py'),
       os.path.join(ROOT, '_tools', 'real_test', 'voice_dialog.py')]


def md5name(text):
    return hashlib.md5(text.encode('utf-8')).hexdigest() + '.wav'


def main():
    have = set(os.listdir(CACHE)) if os.path.isdir(CACHE) else set()
    phrases = []
    for path in SRC:
        if not os.path.exists(path):
            continue
        src = io.open(path, encoding='utf-8').read()
        for m in re.finditer(r"say\(u'([^'%]+)'\)", src):
            phrases.append(m.group(1))
        for m in re.finditer(r"= u'([^'%]+)'", src):
            phrases.append(m.group(1))
    phrases = [p for p in dict((p, None) for p in phrases) if p.strip()]
    hit, miss = [], []
    for p in phrases:
        (hit if md5name(p) in have else miss).append(p)
    print(u'缓存里共 %d 条；我们的静态播报 %d 条，命中 %d 条，缺 %d 条'
          % (len(have), len(phrases), len(hit), len(miss)))
    print(u'')
    if hit:
        print(u'--- 已有（可直接离线播报）---')
        for p in sorted(hit):
            print(u'  ✓ %s' % p)
    print(u'')
    print(u'--- 缺（需要录/合成后补进缓存）---')
    for p in sorted(miss):
        print(u'  ✗ %s' % p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
