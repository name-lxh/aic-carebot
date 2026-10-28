# -*- coding: utf-8 -*-
"""Offline cache planning compatible with TALOS UTF-8 MD5 WAV naming."""
from __future__ import unicode_literals
import hashlib
import os
import re

PHRASES = ['请说哈哈唤醒我，可以询问日期时间和传感器数据，说结束语音交互进入跌倒检测', '语音交互已结束，现在开始跌倒检测'] + ['问答结束，现在开始跌倒检测，请保持上半身在摄像头画面内'] + ['房间巡视完成，正在前往客厅交互点', '请说哈哈唤醒我，可以询问日期时间和传感器数据', '问答结束，现在开始跌倒检测，请先站在摄像头前', '检测到疑似跌倒，你还好吗', '演示模式，现在模拟拨打幺二零，未实际拨打电话', '报警演示完成，请离开返程通道，我会等待通道畅通后返回起点'] + [
    '巡视任务已完成，可以启动语音交互',
    '正在打开卧室窗帘', '窗帘开启位置已确认', '窗帘控制尚未启用，继续任务', '窗帘开启状态未确认，继续任务',
    '主人欢迎回家，这里智慧养老机器人将为你服务',
    '语音系统就绪', '开始执行任务', '正在前往客厅', '已到达客厅',
    '正在识别二维码', '二维码识别成功', '二维码识别失败', '任务已停止',
    '未识别到二维码', '按默认顺序巡视',
    '巡视顺序为', '书房', '厨房', '卧室',
    '正在检测客厅环境', '光照强度', '勒克斯', '温度', '摄氏度', '湿度', '百分之',
    '光照数据暂不可用', '温度数据暂不可用', '湿度数据暂不可用',
    '正在前往人脸识别点', '正在进行人脸识别', '未识别到主人',
    '识别成功', '欢迎主人回家',
    '正在前往书房', '已到达书房', '正在前往厨房', '已到达厨房',
    '正在前往卧室', '已到达卧室',
    '正在检测厨房环境', '燃气浓度为爆炸下限的百分之',
    '燃气浓度数据暂不可用', '燃气报警器状态正常', '检测到燃气报警',
    '燃气报警状态暂不可用', '正在检测漏水状态', '未检测到漏水',
    '检测到漏水', '请及时处理', '漏水传感器未就绪',
    '正在打开卧室电灯', '电灯插座开启状态已确认', '电灯插座开启未确认',
    '正在打开厨房风扇', '风扇插座开启状态已确认', '风扇插座开启未确认',
    '正在返回起点', '已返回起点', '本次路线结束',
    '系统就绪', '等待启动指令',
] + list('零一二三四五六七八九十百千万点负')
PHRASES += [u'\u68c0\u6d4b\u5230\u7591\u4f3c\u8dcc\u5012\uff0c\u8bf7\u7acb\u5373\u62e8\u6253\u6025\u6551\u7535\u8bdd', u'\u8dcc\u5012\u68c0\u6d4b\u7ed3\u675f', u'\u8bf7\u79bb\u5f00\u8fd4\u7a0b\u901a\u9053\uff0c\u6211\u4f1a\u5728\u901a\u9053\u7545\u901a\u540e\u8fd4\u56de\u8d77\u70b9', u'\u561f\u561f\u561f']
PHRASES = list(dict((p, None) for p in PHRASES))
DIGITS = '零一二三四五六七八九'


def filename(text):
    return hashlib.md5(text.encode('utf-8')).hexdigest() + '.wav'


def integer_cn(n):
    if not 0 <= n <= 99999999:
        raise ValueError('Number outside voice range')
    if n < 10:
        return DIGITS[n]
    if n < 20:
        return '十' + (DIGITS[n - 10] if n > 10 else '')
    if n >= 10000:
        high, low = divmod(n, 10000)
        return integer_cn(high) + '万' + (('零' if low < 1000 else '') + integer_cn(low) if low else '')
    out, zero = '', False
    for value, name in ((1000, '千'), (100, '百'), (10, '十'), (1, '')):
        digit, n = divmod(n, value)
        if digit:
            if zero:
                out += '零'
                zero = False
            out += DIGITS[digit] + name
        elif out:
            zero = True
    return out


def number_cn(value):
    sign = '负' if value.startswith('-') else ''
    whole, dot, fraction = value.lstrip('-').partition('.')
    return sign + integer_cn(int(whole)) + ('点' + ''.join(DIGITS[int(c)] for c in fraction) if dot else '')


def plan(text):
    """All-or-nothing plan: never silently drop an unknown word or number."""
    try:
        unicode_type = unicode
    except NameError:
        unicode_type = str
    if not isinstance(text, unicode_type):
        text = text.decode('utf-8')
    if len(text) > 400:
        raise ValueError('Speech text too long')
    out = []
    tokens = sorted(PHRASES, key=len, reverse=True)
    while text:
        m = re.match(r'^[\s，。！？,!?；;、：:]+', text)
        if m:
            text = text[m.end():]
            continue
        m = re.match(r'^-?\d+(?:\.\d+)?', text)
        if m:
            out.extend(list(number_cn(m.group(0))))
            text = text[m.end():]
            continue
        found = next((p for p in tokens if text.startswith(p)), None)
        if found is None:
            raise ValueError('No cached phrase for: ' + text)
        out.append(found)
        text = text[len(found):]
    if not out:
        raise ValueError('Empty speech')
    return out


def cache_paths(text, directory):
    paths = [os.path.join(directory, filename(part)) for part in plan(text)]
    missing = [p for p in paths if not os.path.isfile(p)]
    if missing:
        raise IOError('Missing voice cache: ' + ', '.join(missing))
    return paths
