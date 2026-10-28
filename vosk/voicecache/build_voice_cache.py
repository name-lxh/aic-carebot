#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_voice_cache.py —— 把 phrases.txt 里的句子批量合成为离线 WAV 缓存。

为什么要它：比赛现场没有在线 TTS。趁现在有网，把所有固定播报预生成成 WAV，
运行时只查表 + aplay，零网络、毫秒级。

用法（车上，python3）：
    python3 build_voice_cache.py                 # 生成缺的
    python3 build_voice_cache.py --force         # 全部重生成
    python3 build_voice_cache.py --test "你好"    # 只合成一句试试

实现：百度 gettts（tts_net 一直用的同一个接口）→ mp3 → ffmpeg 转 16k 单声道
      → sox 归一化 → cache/<md5(文本)>.wav，并写 manifest.json。
"""
from __future__ import print_function

import hashlib
import io
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, 'cache')
MANIFEST = os.path.join(HERE, 'manifest.json')
PHRASES = os.path.join(HERE, 'phrases.txt')
TTS_URL = ('https://fanyi.baidu.com/gettts?lan=zh&text={text}'
           '&spd={spd}&source=web')
UA = ('Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 '
      '(KHTML, like Gecko) Chrome/120.0 Safari/537.36')


def md5name(text):
    return hashlib.md5(text.encode('utf-8')).hexdigest() + '.wav'


def load_manifest():
    if os.path.isfile(MANIFEST):
        try:
            return json.load(io.open(MANIFEST, encoding='utf-8'))
        except Exception:
            return {}
    return {}


def save_manifest(data):
    io.open(MANIFEST, 'w', encoding='utf-8', newline='\n').write(
        json.dumps(data, ensure_ascii=False, indent=1, sort_keys=True))


def fetch_mp3(text, spd=5, timeout=15):
    url = TTS_URL.format(text=urllib.parse.quote(text), spd=spd)
    req = urllib.request.Request(url, headers={'User-Agent': UA,
                                               'Referer': 'https://fanyi.baidu.com/'})
    data = urllib.request.urlopen(req, timeout=timeout).read()
    if len(data) < 500:
        raise RuntimeError('返回太短（%d 字节），可能被限流' % len(data))
    return data


def to_wav(mp3_bytes, wav_path):
    fd, mp3_path = tempfile.mkstemp(suffix='.mp3')
    os.close(fd)
    try:
        with open(mp3_path, 'wb') as fh:
            fh.write(mp3_bytes)
        cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-i', mp3_path,
               '-ar', '16000', '-ac', '1', '-sample_fmt', 's16', wav_path]
        subprocess.check_call(cmd)
        # 响度归一化（sox 有就用，失败不影响可用性）
        try:
            subprocess.check_call(['sox', wav_path, wav_path + '.n.wav', 'norm', '-3'],
                                  stderr=subprocess.DEVNULL)
            os.replace(wav_path + '.n.wav', wav_path)
        except Exception:
            pass
    finally:
        try:
            os.unlink(mp3_path)
        except OSError:
            pass


def duration(path):
    out = subprocess.check_output(['soxi', '-D', path]).decode().strip() \
        if os.path.exists('/usr/bin/soxi') else ''
    try:
        return round(float(out), 2)
    except Exception:
        return None


def build_one(text, force=False, spd=5):
    name = md5name(text)
    path = os.path.join(CACHE, name)
    if os.path.isfile(path) and not force:
        return 'skip'
    mp3 = fetch_mp3(text, spd=spd)
    to_wav(mp3, path)
    return 'ok'


def main():
    if not os.path.isdir(CACHE):
        os.makedirs(CACHE)
    args = sys.argv[1:]
    if '--test' in args:
        text = args[args.index('--test') + 1]
        build_one(text, force=True)
        print('已生成：', os.path.join(CACHE, md5name(text)))
        return 0
    force = '--force' in args
    texts = [line.strip() for line in io.open(PHRASES, encoding='utf-8')
             if line.strip()]
    manifest = load_manifest()
    ok = skip = fail = 0
    for i, text in enumerate(texts, 1):
        try:
            state = build_one(text, force=force)
        except Exception as exc:
            fail += 1
            print('[%d/%d] 失败 %s：%r' % (i, len(texts), text, exc))
            continue
        if state == 'skip':
            skip += 1
            # ★ 即使音频已存在（可能来自既有缓存），也要登记进 manifest，
            #   否则 tts_play 的切词表里没有这条，拼接时会退化成 espeak。
            text_path = os.path.join(CACHE, md5name(text))
            manifest[text] = {'file': md5name(text), 'seconds': duration(text_path),
                              'source': 'existing', 'at': time.strftime('%Y-%m-%d %H:%M:%S')}
        else:
            ok += 1
            manifest[text] = {'file': md5name(text), 'seconds': duration(
                os.path.join(CACHE, md5name(text))), 'source': 'baidu_gettts',
                'at': time.strftime('%Y-%m-%d %H:%M:%S')}
            print('[%d/%d] 生成 %s' % (i, len(texts), text))
    save_manifest(manifest)
    print('完成：新生成 %d，已存在 %d，失败 %d；缓存共 %d 个文件'
          % (ok, skip, fail, len(os.listdir(CACHE))))
    return 0 if fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
