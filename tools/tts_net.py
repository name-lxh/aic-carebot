# -*- coding: utf-8 -*-
"""tts_net.py —— 在线中文 TTS 播放节点（替代 espeak / 坏掉的讯飞离线引擎）

## 为什么

车上的两条合成路径都不行：
  · 讯飞离线 TTS：QTTSSessionBegin 报 10107，合成出来是 0 字节 WAV
  · espeak：缺 zh 词典（`Full dictionary is not installed for 'zh'`），中文含糊不清

所以改成走在线 TTS（百度翻译的 gettts 接口，无需密钥，实测返回 13 KB MPEG-1 Layer3）：
    文本 -> GET gettts -> /tmp/carebot_tts/net_<n>.mp3 -> mpv 播放

## 接口

    订阅 /voice/say   std_msgs/String   要播报的文本（UTF-8）
    发布 /voice/status std_msgs/String   播放状态（便于排查）

## 用法

    python2 ~/Hajimi/LXH/_tools/tts_net.py
    # 然后照常往 /voice/say 发文本即可
"""
from __future__ import print_function

import io
import os
import Queue
import subprocess
import sys
import time
import tempfile
import threading
import urllib
import urllib2


# ---- 输出编码兜底：重定向到文件时 locale 可能是 ASCII ----
import codecs as _codecs
for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, _codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

import rospy
from std_msgs.msg import String

TTS_URL = 'https://fanyi.baidu.com/gettts?lan=zh&spd=5&source=web&text=%s'
UA = 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120 Safari/537.36'
TMP_DIR = '/tmp/carebot_tts'
# 这台车的 ffplay 不支持 -volume，改用 mpv（实测能出声）
def pick_audio_device():
    """自动挑一块**能放音**的声卡。

    这台车的 /etc/asound.conf 把 default 指到了 card 0，而 card 0 只有 Mic
    （纯采集），真正的喇叭在另一块卡上 —— 结果就是"播了但永远没声音"。
    （2026-10-21 现场排查确认）

    规则：
      1. 环境变量 TTS_AUDIO_DEVICE 优先（换机器时直接指定，比如 alsa/plughw:2,0）
      2. 否则从 `aplay -l` 里找第一块名字带 Analog 的卡
      3. 都不行就用默认设备
    """
    env = os.environ.get('TTS_AUDIO_DEVICE')
    if env:
        return env
    try:
        out = subprocess.Popen(['aplay', '-l'], stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL).communicate()[0]
        for line in out.decode('utf-8', 'ignore').splitlines():
            if line.startswith('card') and 'Analog' in line and 'device' in line:
                head = line.split(':')[0]           # "card 2"
                dev = line.split('device')[1].split(':')[0].strip()
                card = head.split()[1]
                return 'alsa/plughw:%s,%s' % (card, dev)
    except Exception:
        pass
    return None


AUDIO_DEV = pick_audio_device()
PLAYER = ['mpv', '--no-video', '--really-quiet', '--volume=95']
if AUDIO_DEV:
    PLAYER += ['--audio-device=' + AUDIO_DEV]
TIMEOUT = 8


class TtsNet(object):
    def __init__(self):
        if not os.path.isdir(TMP_DIR):
            os.makedirs(TMP_DIR)
        self.q = Queue.Queue(maxsize=32)
        self.status_pub = rospy.Publisher('/voice/status', String, queue_size=10)
        rospy.Subscriber('/voice/say', String, self.on_say, queue_size=32)
        self.seq = 0
        threading.Thread(target=self.worker).start()
        self._status(u'在线 TTS 就绪（百度 gettts + mpv）')

    def _status(self, text):
        print(u'  [tts] %s' % text)
        try:
            self.status_pub.publish(String(data=text.encode('utf-8')))
        except Exception:
            pass

    def on_say(self, msg):
        text = msg.data
        if isinstance(text, str):
            text = text.decode('utf-8', 'ignore')
        text = text.strip()
        if not text:
            return
        if self.q.full():
            self._status(u'队列满了，丢弃一条播报')
            return
        self.q.put(text)

    def synth(self, text):
        self.seq += 1
        path = os.path.join(TMP_DIR, 'net_%04d.mp3' % self.seq)
        url = TTS_URL % urllib.quote(text.encode('utf-8'))
        req = urllib2.Request(url, headers={'User-Agent': UA})
        data = urllib2.urlopen(req, timeout=TIMEOUT).read()
        if len(data) < 1000:
            raise RuntimeError('返回数据过小(%d B)，可能被限流' % len(data))
        with io.open(path, 'wb') as f:
            f.write(data)
        return path

    def worker(self):
        while not rospy.is_shutdown():
            try:
                text = self.q.get(timeout=0.5)
            except Queue.Empty:
                continue
            try:
                path = self.synth(text)
                self._status(u'播放: %s' % text[:40])
                subprocess.call(PLAYER + [path])
                try:
                    os.remove(path)
                except Exception:
                    pass
            except Exception as exc:
                self._status(u'合成/播放失败: %r' % exc)
                # 网络挂了就退回本地 espeak，至少别哑掉
                try:
                    wav = tempfile.mktemp(suffix='.wav', dir=TMP_DIR)
                    subprocess.call(['espeak', '-v', 'zh', '-s', '120', '-w', wav, text])
                    subprocess.call(['aplay', '-q', wav])
                except Exception:
                    pass
            time.sleep(0.05)


def main():
    rospy.init_node('tts_net', anonymous=False)
    TtsNet()
    rospy.spin()


if __name__ == '__main__':
    main()
