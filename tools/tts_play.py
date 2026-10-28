#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""tts_play.py —— 离线播报节点：/voice/say 文本 → 本地 WAV 查表播放，不联网。

为什么替换 tts_net：比赛现场没有在线 TTS。所有固定播报已预生成为
`_tools/voicecache/cache/<md5(文本)>.wav`（build_voice_cache.py 生成），
运行时只做"文本 → 若干缓存片段 → aplay 顺序播放"，毫秒级、零网络。

处理顺序：
  1) 整句命中缓存 → 直接播（最快）
  2) 带数字/单位的句子 → 归一化（阿拉伯数字转中文、lx/°C/% 转中文单位）
     → 用最长匹配把句子切成缓存片段 → 逐段播
  3) 片段缺失 → 该片段用 espeak 兜底（会打日志，提示补录）

订阅：/voice/say (std_msgs/String)
日志：/tmp/f_tts.log（m40 启动时重定向）
"""
from __future__ import print_function

import codecs
import hashlib
import io
import json
import os
import Queue
import re
import subprocess
import sys
import threading
import time

import rospy
from std_msgs.msg import String

# 车上 stdout 重定向到文件时默认是 ascii，中文会炸；统一改成 UTF-8
try:
    if not getattr(sys.stdout, 'encoding', None) or             'utf' not in sys.stdout.encoding.lower():
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout)
        sys.stderr = codecs.getwriter('utf-8')(sys.stderr)
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, 'voicecache', 'cache')
MANIFEST = os.path.join(HERE, 'voicecache', 'manifest.json')

DIGITS = u'零一二三四五六七八九'
# ★ 2026-10-23：默认**关闭 espeak 兜底**。现场反馈"两个声音"，根因就是
#   缓存没覆盖到的句子退到 espeak（机器音）播放。现在缺失只记日志不发声，
#   保证全程只有一种音色。需要时 export TTS_ESPEAK=1 再开。
ESPEAK_FALLBACK = os.environ.get('TTS_ESPEAK', '0') == '1'

UNIT_REPLACE = [
    (u'°C', u'摄氏度'), (u'℃', u'摄氏度'), (u'lx', u'勒克斯'),
    (u'LX', u'勒克斯'), (u'%', u'百分之'), (u'ppm', u'ppm'),
]


def md5name(text):
    return hashlib.md5(text.encode('utf-8')).hexdigest() + '.wav'


def integer_cn(n):
    if n < 10:
        return DIGITS[n]
    if n < 20:
        return u'十' + (DIGITS[n - 10] if n > 10 else u'')
    if n >= 10000:
        high, low = divmod(n, 10000)
        return integer_cn(high) + u'万' + (integer_cn(low) if low else u'')
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
    text = re.sub(u'[\\s，。！？、：；,\.!?:;（）()【】\\[\\]"“”\'‘’]+', u'', text)
    return text


class OfflineVoice(object):
    def __init__(self):
        self.texts = {}
        if os.path.isfile(MANIFEST):
            try:
                data = json.load(io.open(MANIFEST, encoding='utf-8'))
                self.texts = dict((t, CACHE + '/' + v['file'])
                                  for t, v in data.items())
            except Exception as exc:
                print(u'[tts] 读 manifest 失败：%r' % exc)
        if not self.texts:
            # 没 manifest 就扫缓存目录，按"文本=?"无法反查，只能整句命中
            print(u'[tts] 警告：manifest.json 为空，只能整句命中')
        # 键也要按同样规则归一化（去掉标点），否则"当前客厅环境："里的冒号
        # 在归一化后对不上，会退化成 espeak。
        pairs = {}
        for t, path in self.texts.items():
            pairs.setdefault(normalise(t), path)
        self.keys = sorted(pairs.keys(), key=len, reverse=True)
        self.paths = pairs
        self.queue = Queue.Queue()
        self.worker = threading.Thread(target=self._run)
        self.worker.daemon = True
        self.worker.start()

    # ---- 播放 ----
    def _aplay(self, path):
        try:
            subprocess.call(['aplay', '-q', path])
            return True
        except Exception as exc:
            print(u'[tts] aplay 失败 %s: %r' % (path, exc))
            return False

    def _espeak(self, text):
        if not ESPEAK_FALLBACK:
            print(u'[tts] 缓存缺失（已静默，保持单一音色）：%s' % text)
            return
        print(u'[tts] 缓存缺失，espeak 兜底：%s' % text)
        try:
            subprocess.call(['espeak', '-v', 'zh', '-s', '150', text])
        except Exception as exc:
            print(u'[tts] espeak 也失败：%r' % exc)

    def _tokenise(self, text):
        """最长匹配切分；返回 [(kind, content)]，kind ∈ {'cache','miss'}。"""
        out, i, miss = [], 0, u''
        while i < len(text):
            hit = None
            for key in self.keys:
                if key and text.startswith(key, i):
                    hit = key
                    break
            if hit:
                if miss:
                    out.append(('miss', miss))
                    miss = u''
                out.append(('cache', hit))
                i += len(hit)
            else:
                miss += text[i]
                i += 1
        if miss:
            out.append(('miss', miss))
        return out

    def say(self, text):
        text = (text or u'').strip()
        if not text:
            return
        whole = md5name(text)
        whole_path = os.path.join(CACHE, whole)
        if os.path.isfile(whole_path):
            print(u'[tts] 播报（整句）：%s' % text)
            self._aplay(whole_path)
            return
        norm = normalise(text)
        tokens = self._tokenise(norm)
        print(u'[tts] 播报（拼接 %d 段）：%s' % (len(tokens), text))
        # 全部命中缓存时先拼成一个 wav 再播，避免每段之间 ~80 ms 的 aplay 启动间隔
        if tokens and all(k == 'cache' and os.path.isfile(self.paths.get(c, ''))
                          for k, c in tokens):
            paths = [self.paths[c] for k, c in tokens]
            merged = os.path.join('/tmp', 'tts_merge_%d.wav' % int(time.time() * 1000))
            try:
                subprocess.check_call(['sox'] + paths + [merged],
                                      stderr=subprocess.DEVNULL)
                self._aplay(merged)
                return
            except Exception:
                pass
            finally:
                try:
                    os.unlink(merged)
                except OSError:
                    pass
        for kind, content in tokens:
            if kind == 'cache':
                path = self.paths.get(content)
                if path and os.path.isfile(path):
                    self._aplay(path)
                else:
                    self._espeak(content)
            else:
                self._espeak(content)

    def _run(self):
        while not rospy.is_shutdown():
            try:
                text = self.queue.get(timeout=0.2)
            except Queue.Empty:
                continue
            try:
                self.say(text)
            except Exception as exc:
                print(u'[tts] 播报异常：%r' % exc)


def main():
    rospy.init_node('tts_play', anonymous=True)
    voice = OfflineVoice()
    rospy.Subscriber('/voice/say', String, lambda m: voice.queue.put(
        m.data.decode('utf-8') if isinstance(m.data, str) else m.data),
        queue_size=20)
    wavs = len([f for f in os.listdir(CACHE) if f.endswith('.wav')])         if os.path.isdir(CACHE) else 0
    print(u'[tts] 离线播报就绪：%d 条短语，缓存 %d 个 wav'
          % (len(voice.texts), wavs))
    rospy.spin()


if __name__ == '__main__':
    main()
