# -*- coding: utf-8 -*-
"""intent_match.py —— 语音识别结果 -> 意图（**精准匹配版**）

## 为什么改成精准

2026-10-22 现场：模糊匹配太松，说"我有点热"被匹配成"到厨房"（残留了
"厨房"两个字），连着两次误触发导航。
比赛时会说**具体、固定的指令**，所以把词表收窄到每个动作 1 条说法，
阈值提到 0.80，宁可"没听清"也不要误执行。

## 匹配规则（三个分取最大）

    * 字符集合 Jaccard（抗漏字/多字）
    * difflib 编辑距离（抗认错个别字）
    * 子串包含（短句完整出现在长句里）—— 权重压低，避免"厨房"命中"导航到厨房"

阈值 0.80：
    "查询室内温度" 与 "查询室内温"   = 0.91  ✅ 通过（少一个字）
    "我有点热"     与 "我有点执"     = 0.75  ❌ 拒绝（短句错一个字就不认）
    "厨房"         与 "导航到厨房"   = 0.73  ❌ 拒绝（必须说完整指令）

每个设备类意图还要求文本里出现**对象词**（灯/风扇/窗帘/热），
否则"打开O"这类噪声会被误判成开灯。
"""
from __future__ import unicode_literals

import difflib

try:
    text_type = unicode
except NameError:
    text_type = str

# ---------------------------------------------------------------------------
# 精准词表：每个意图 1 条（唤醒/结束也各 1 条）
# ---------------------------------------------------------------------------
VOCAB = [
    ('wake', [u'老张同学']),
    ('temp', [u'查询室内温度']),
    ('goto:C', [u'导航到厨房']),
    ('goto:D', [u'导航到卧室']),
    ('goto:B', [u'导航到书房']),
    ('goto:living', [u'导航到客厅']),
    ('hot', [u'我有点热']),
    ('fan_on', [u'打开厨房风扇']),
    ('fan_off', [u'关闭厨房风扇']),
    ('light_on', [u'打开卧室灯']),
    ('light_off', [u'关闭卧室灯']),
    ('curtain_open', [u'打开卧室窗帘']),
    ('curtain_close', [u'关闭卧室窗帘']),
    ('stop', [u'结束语音交互']),
]

# 设备类意图的必需对象词（防噪声误判）
REQUIRED = {
    'light_on': u'灯', 'light_off': u'灯',
    'fan_on': u'风扇', 'fan_off': u'风扇',
    'curtain_open': u'窗帘', 'curtain_close': u'窗帘',
    'hot': u'热',
}

THRESHOLD = 0.80


def _jac(a, b):
    sa, sb = set(a), set(b)
    if not sa or not sb:
        return 0.0
    return len(sa & sb) / float(len(sa | sb))


def _seq(a, b):
    return difflib.SequenceMatcher(None, a, b).ratio()


def _sub(a, b):
    """子串包含，权重压低（0.55 起），避免短词命中长句。"""
    if not a or not b:
        return 0.0
    short, long_ = (a, b) if len(a) <= len(b) else (b, a)
    if short in long_:
        return 0.55 + 0.30 * (len(short) / float(len(long_)))
    return 0.0


def score(text, phrase):
    return max(_jac(text, phrase), _seq(text, phrase), _sub(text, phrase))


def match(text, threshold=THRESHOLD, return_all=False):
    """识别文本 -> (意图, 命中的词, 分数)；没命中返回 None。"""
    if not text:
        return None
    text = text.strip()
    if not text:
        return None
    best = None
    allhits = []
    for intent, phrases in VOCAB:
        need = REQUIRED.get(intent)
        if need and need not in text:
            continue
        for p in phrases:
            s = score(text, p)
            allhits.append((s, intent, p))
            if best is None or s > best[0]:
                best = (s, intent, p)
    if return_all:
        allhits.sort(reverse=True)
        return allhits[:5]
    if best and best[0] >= threshold:
        return (best[1], best[2], round(best[0], 3))
    return None


# ---------------------------------------------------------------- 本地自测
if __name__ == '__main__':
    cases = [
        (u'老张同学', 'wake'),
        (u'查询室内温度', 'temp'),
        (u'查询室内温', 'temp'),        # 少一个字，0.91 应通过
        (u'导航到厨房', 'goto:C'),
        (u'导航到卧室', 'goto:D'),
        (u'导航到书房', 'goto:B'),
        (u'导航到客厅', 'goto:living'),
        (u'我有点热', 'hot'),
        (u'打开厨房风扇', 'fan_on'),
        (u'关闭厨房风扇', 'fan_off'),
        (u'打开卧室灯', 'light_on'),
        (u'关闭卧室灯', 'light_off'),
        (u'打开卧室窗帘', 'curtain_open'),
        (u'关闭卧室窗帘', 'curtain_close'),
        (u'结束语音交互', 'stop'),
        # ---- 以下必须判不出来（精准版的关键）----
        (u'厨房', None),
        (u'到厨房', None),
        (u'带我去厨房', None),
        (u'打开O', None),
        (u'关闭P', None),
        (u'小车P', None),
        (u'我有点执', None),
        (u'', None),
    ]
    bad = 0
    print('  %-12s %-12s %-12s %s' % (u'识别文本', u'期望', u'实际', u'分数'))
    for text, want in cases:
        r = match(text)
        got = r[0] if r else None
        sc = r[2] if r else 0.0
        ok = (got == want)
        bad += 0 if ok else 1
        print('  %-12s %-12s %-12s %.2f %s'
              % (text or u'(空)', str(want), str(got), sc, u'' if ok else u'  <-- 不符'))
    print('')
    print(u'  通过 %d / %d' % (len(cases) - bad, len(cases)))
