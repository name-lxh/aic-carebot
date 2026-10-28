# -*- coding: utf-8 -*-
"""从任务代码里抽出所有播报句，生成离线语音缓存的待录清单 phrases.txt。

用法（本地）： python _tools/voicecache/extract_phrases.py
输出： _tools/voicecache/phrases.txt（每行一句，已去重）
"""
from __future__ import print_function

import io
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = [os.path.join(ROOT, '_tools', 'real_test', 'm36_mission_v5.py'),
       os.path.join(ROOT, '_tools', 'real_test', 'voice_dialog.py')]
OUT = os.path.join(ROOT, '_tools', 'voicecache', 'phrases.txt')

# 手工补充：动态句的前缀/单位、数字片段、跌倒与会话用语
EXTRA = [
    # 会话
    u'主人，有什么我能帮你的', u'我在', u'那我先忙了', u'好的，我随时都在',
    u'没听清，请再说一次', u'时间到，我先去忙了',
    # 动态句前缀与单位
    u'当前客厅环境：', u'光照强度', u'温度', u'湿度', u'勒克斯', u'摄氏度',
    u'百分之', u'当前室内温度', u'您当前的心率是', u'次每分',
    u'厨房烟雾浓度', u'烟雾状态', u'正常', u'漏水', u'异常',
    # 跌倒
    u'开始跌倒检测', u'检测到疑似跌倒，请及时处理', u'未检测到跌倒，环境正常',
    u'跌倒检测未启动，继续返回起点', u'请保持静止，正在检测',
    # 数字与标点
    u'零', u'一', u'二', u'三', u'四', u'五', u'六', u'七', u'八', u'九',
    u'十', u'百', u'千', u'万', u'点', u'负', u'百分之',
]


def main():
    found = []
    for path in SRC:
        if not os.path.exists(path):
            continue
        src = io.open(path, encoding='utf-8').read()
        # say(u'...') / say(u"...")  / _say(u'...')
        for m in re.finditer(r"say\(u'([^']*)'", src):
            s = m.group(1).strip()
            if s and '%' not in s and '\\' not in s:
                found.append(s)
        for m in re.finditer(r"_say\(u'([^']*)'", src):
            s = m.group(1).strip()
            if s and '%' not in s and '\\' not in s:
                found.append(s)
        # 常量： XXX = u'...'
        for m in re.finditer(r"^[A-Z_]+ = u'([^']*)'", src, re.M):
            s = m.group(1).strip()
            if s and '%' not in s:
                found.append(s)
    lines = []
    for s in found + EXTRA:
        if s and s not in lines:
            lines.append(s)
    io.open(OUT, 'w', encoding='utf-8', newline='\n').write(u'\n'.join(lines) + u'\n')
    print(u'写出 %d 条 -> %s' % (len(lines), OUT))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
