#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""asr_relay.py —— 把 Vosk(py3) 的识别结果转成 ROS 话题 /voice_words。

结构：
    vosk_asr.py（py3 + vosk + arecord 直采）
        │ stdout 每行一个 JSON
        ▼
    asr_relay.py（py2 + rospy）
        │ 置信度过滤后
        ▼
    /voice_words (std_msgs/String)  ← 对话/意图匹配照旧订阅这里

和讯飞版相比：不依赖 BNF 文法、不需要 pulseaudio、置信度自己掌握，
噪声片段（1 个字、置信度低）直接丢，不往上送。
"""
from __future__ import print_function

import codecs
import json
import os
import select
import subprocess
import sys
import threading
import time

import rospy
from std_msgs.msg import String

try:
    if not getattr(sys.stdout, 'encoding', None) or \
            'utf' not in sys.stdout.encoding.lower():
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout)
        sys.stderr = codecs.getwriter('utf-8')(sys.stderr)
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
PY3 = '/home/ub/talos_venv/bin/python'
ASR = os.path.join(HERE, 'vosk_asr.py')
MIN_CONF = 0.60
MIN_CHARS = 2


class Relay(object):
    def __init__(self, model=None, device=None, grammar=None, phrases=None,
                 min_conf=MIN_CONF, debug=False):
        # 回声门控：车自己播报时（以及播完 1.2 s 内）不接受识别结果，
        # 否则 Vosk 会把"没听清，请再说一次"当成用户说话，形成死循环。
        self.mute_until = 0.0
        self.mute_lock = threading.Lock()
        rospy.Subscriber('/voice/say', String, self._on_say, queue_size=20)
        self._extra_args = []
        cmd = [PY3, '-u', ASR]
        if model:
            cmd += ['--model', model]
            self._extra_args += ['--model', model]
        if device:
            cmd += ['--device', device]
            self._extra_args += ['--device', device]
        if phrases:
            cmd += ['--phrases', phrases]
            self._extra_args += ['--phrases', phrases]
        elif grammar:
            cmd += ['--grammar', grammar]
            self._extra_args += ['--grammar', grammar]
        env = os.environ.copy()
        env.pop('PYTHONPATH', None)
        env.pop('PYTHONHOME', None)
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0,
                                     env=env)
        self.buf = b''
        self.min_conf = min_conf
        self.debug = debug
        self.pub = rospy.Publisher('/voice_words', String, queue_size=10)
        self.ready = False

    def _on_say(self, msg):
        try:
            text = msg.data.decode('utf-8') if isinstance(msg.data, str) else msg.data
        except Exception:
            text = u''
        dur = max(1.0, len(text or u'') * 0.28) + 1.2
        with self.mute_lock:
            self.mute_until = max(self.mute_until, time.time() + dur)

    def _muted(self):
        with self.mute_lock:
            return time.time() < self.mute_until

    def _line(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if b'\n' in self.buf:
                line, self.buf = self.buf.split(b'\n', 1)
                return line
            if self.proc.poll() is not None:
                err = self.proc.stderr.read()[-400:] if self.proc.stderr else b''
                raise RuntimeError(u'asr 退出：%s' % err.decode('utf-8', 'replace'))
            if select.select([self.proc.stdout], [], [], 0.1)[0]:
                data = os.read(self.proc.stdout.fileno(), 65536)
                if not data:
                    raise RuntimeError(u'asr 输出结束')
                self.buf += data
        return None

    def spin(self):
        print(u'[asr] 等待 Vosk 就绪…')
        while not rospy.is_shutdown():
            line = self._line(1.0)
            if line is None:
                continue
            try:
                msg = json.loads(line.decode('utf-8'))
            except Exception:
                continue
            if msg.get('ready'):
                self.ready = True
                print(u'[asr] Vosk 就绪：%s（模型 %s，加载 %.2fs）'
                      % (msg.get('device'), msg.get('model'),
                         msg.get('load_s') or 0))
                continue
            if msg.get('error'):
                print(u'[asr] 错误：%s %s（3 s 后自动重启识别进程）'
                      % (msg.get('error'), msg.get('detail', '')))
                self._restart_child()
                continue
            # Vosk 输出是"词 词 词"（空格分隔），中文要拼回整句再交给意图匹配
            text = (msg.get('text') or u'').replace(' ', u'').strip()
            conf = float(msg.get('conf') or 0)
            if not text:
                continue
            keep = (conf >= self.min_conf and len(text) >= MIN_CHARS
                    and not self._muted())
            print(u'[asr] %s [%s] conf=%.2f' % (
                u'识别' if keep else u'丢弃', text, conf))
            if keep:
                self.pub.publish(String(data=text.encode('utf-8')))

    def _restart_child(self):
        """麦克风被抢/arecord 掉线时，自动重启识别子进程（不退出中继）。"""
        try:
            if self.proc.poll() is None:
                self.proc.terminate()
                self.proc.wait(timeout=2)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        time.sleep(3.0)
        cmd = [PY3, '-u', ASR] + self._extra_args
        env = os.environ.copy()
        env.pop('PYTHONPATH', None)
        env.pop('PYTHONHOME', None)
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0,
                                     env=env)
        self.buf = b''
        self.ready = False
        print(u'[asr] 已重启识别进程')

    def close(self):
        try:
            if self.proc.poll() is None:
                self.proc.terminate()
                self.proc.wait(timeout=3)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass


def main():
    rospy.init_node('asr_relay', anonymous=True)
    model = rospy.get_param('~model', '')
    device = rospy.get_param('~device', '')
    grammar = rospy.get_param('~grammar', '')
    phrases = rospy.get_param('~phrases', '')
    min_conf = float(rospy.get_param('~min_conf', MIN_CONF))
    relay = Relay(model=model or None, device=device or None,
                  grammar=grammar or None, phrases=phrases or None,
                  min_conf=min_conf)
    try:
        relay.spin()
    finally:
        relay.close()


if __name__ == '__main__':
    main()
