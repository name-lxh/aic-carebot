#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vosk_asr.py —— Vosk 离线识别（py3），麦克风直采 ALSA，逐行输出 JSON。

为什么不用讯飞：噪声下置信度窗口太窄（真指令 8~11 分、噪声 0~7 分），
封闭 BNF 改一次要重装重启 22 s，还要独占麦克风。Vosk 全离线、可语法受限，
识别结果与置信度直接给出来，由上层决定用不用。

输出（stdout，每行一个 JSON）：
    {"ready": true, "model": "...", "device": "..."}
    {"text": "打开卧室灯", "conf": 0.93, "words": 4, "t": 12.3}
    {"partial": "打开卧", "t": 1.2}          # 仅 --partial 时

用法：
    /home/ub/talos_venv/bin/python -u vosk_asr.py [--device plughw:CARD=S,DEV=0]
                                                  [--grammar grammar.json]
                                                  [--partial]
"""
from __future__ import print_function

import argparse
import json
import os
import subprocess
import sys
import time

DEFAULT_MODEL = ('/home/ub/tianxing_team/jie_voice/data/models/'
                 'vosk-model-small-cn-0.22')
DEFAULT_DEVICE = 'plughw:CARD=S,DEV=0'


def emit(obj):
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + '\n')
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--device', default=DEFAULT_DEVICE)
    ap.add_argument('--grammar', default='')
    ap.add_argument('--phrases', default='')   # 每行一句，自动生成 Vosk 语法
    ap.add_argument('--partial', action='store_true')
    ap.add_argument('--rate', type=int, default=16000)
    args = ap.parse_args()

    if not os.path.isdir(args.model):
        emit({'error': 'model not found: %s' % args.model})
        return 2

    from vosk import Model, KaldiRecognizer, SetLogLevel
    SetLogLevel(-1)
    t0 = time.time()
    model = Model(args.model)
    load = time.time() - t0

    grammar = ''
    if args.phrases:
        phrases = [l.strip() for l in open(args.phrases, encoding='utf-8')
                   if l.strip()]
        import json as _json
        grammar = _json.dumps(phrases, ensure_ascii=False)
        emit({'grammar_phrases': len(phrases)})
    elif args.grammar:
        with open(args.grammar, encoding='utf-8') as fh:
            grammar = fh.read()
    if grammar:
        rec = KaldiRecognizer(model, args.rate, grammar)
    else:
        rec = KaldiRecognizer(model, args.rate)
    rec.SetWords(True)
    if hasattr(rec, 'SetEndpointerMode'):
        rec.SetEndpointerMode(1)     # 更快收尾，适合问答

    mic = subprocess.Popen(
        ['arecord', '-q', '-D', args.device, '-f', 'S16_LE',
         '-r', str(args.rate), '-c', '1', '-t', 'raw'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

    emit({'ready': True, 'model': os.path.basename(args.model.rstrip('/')),
          'device': args.device, 'load_s': round(load, 2),
          'grammar': bool(args.grammar)})

    started = time.time()
    try:
        while True:
            data = mic.stdout.read(4000)     # 0.125 s
            if not data:
                err = mic.stderr.read()[-300:] if mic.stderr else b''
                emit({'error': 'mic closed', 'detail': err.decode('utf-8', 'replace')})
                return 3
            if rec.AcceptWaveform(data):
                res = json.loads(rec.Result())
                text = (res.get('text') or '').strip()
                words = res.get('result') or []
                if not text:
                    continue
                conf = (sum(float(w.get('conf', 0)) for w in words) / len(words)
                        if words else 0.0)
                emit({'text': text, 'conf': round(conf, 3),
                      'words': len(words), 't': round(time.time() - started, 2)})
            elif args.partial:
                part = json.loads(rec.PartialResult()).get('partial', '')
                if part:
                    emit({'partial': part, 't': round(time.time() - started, 2)})
    except KeyboardInterrupt:
        pass
    finally:
        try:
            mic.terminate()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
