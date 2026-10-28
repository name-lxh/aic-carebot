# -*- coding: utf-8 -*-
"""patch_bnf.py —— 给讯飞文法加"老张同学"唤醒词（带上原文件备份）

BNF 结构：
    <callstart>:<name><choose>;      # 一句话 = 名字 + 动作/地点...
所以"老张同学"这种独立喊话要单开一条规则。
"""
from __future__ import print_function

import io
import os
import shutil
import time

BNF = '/home/ub/omniWheelCareRobot/rosCode/src/xf_mic_asr_offline/config/call.bnf'

WAKE = u'老张同学'

s = io.open(BNF, encoding='utf-8-sig').read()   # 吃掉 BOM
print(u'  原文法 %d 字节' % len(s))

if WAKE in s:
    print(u'  已经有唤醒词，跳过')
else:
    bk = BNF + '.bak_' + time.strftime('%H%M%S')
    shutil.copy2(BNF, bk)
    print(u'  已备份 -> %s' % os.path.basename(bk))

    s2 = s.replace(u'<callstart>:<name><choose>;',
                   u'<callstart>:<name><choose>|<wake>;')
    assert s2 != s, u'没找到 <callstart> 规则'
    s2 = s2.replace(u'<turn>:(左|右)转;',
                    u'<turn>:(左|右)转;\n<wake>:%s|小张同学;' % WAKE)
    io.open(BNF, 'wb').write(s2.encode('utf-8'))
    print(u'  已加入唤醒词：%s' % WAKE)

print(u'')
print(io.open(BNF, encoding='utf-8-sig').read())
