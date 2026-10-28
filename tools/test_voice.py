# -*- coding: utf-8 -*-
"""test_voice.py —— 测试语音识别 + 唤醒词

    python2 test_voice.py [秒数]

行为：
  * 订阅 /voice_words（讯飞离线识别结果）
  * 打印每一句识别到的文本
  * 若包含唤醒词"老张同学" -> 播报"我在"，并开启一段 15 s 的"听命令"窗口
  * 窗口内识别到的内容会标成 [命令]

用于现场验证：
    1) 唤醒词能不能被认出来
    2) 识别延迟多大
    3) 命令词（小车前进/打开电灯/客厅…）能不能认
"""
from __future__ import print_function

import codecs
import sys
import time

import rospy
from std_msgs.msg import String

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

WAKE_WORDS = (u'老张同学', u'小张同学')
LISTEN_AFTER_WAKE = 15.0

state = {'awake_until': 0.0, 'n': 0, 'wake': 0, 'cmd': 0}


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rospy.init_node('test_voice', anonymous=True)
    say_pub = rospy.Publisher('/voice/say', String, queue_size=5)

    def say(text):
        print(u'  [播报] %s' % text)
        try:
            say_pub.publish(String(data=text.encode('utf-8')))
        except Exception:
            pass

    def cb(msg):
        try:
            text = msg.data.decode('utf-8') if isinstance(msg.data, str) else msg.data
        except Exception:
            text = u'?'
        text = text.strip()
        if not text:
            return
        state['n'] += 1
        now = time.time()
        awake = now < state['awake_until']
        tag = u'[命令]' if awake else u'      '
        print(u'  %s %s' % (tag, text))
        if any(w in text for w in WAKE_WORDS):
            state['wake'] += 1
            state['awake_until'] = now + LISTEN_AFTER_WAKE
            print(u'     ★ 唤醒！接下来 %.0f s 内识别到的都当命令' % LISTEN_AFTER_WAKE)
            say(u'我在')
        elif awake:
            state['cmd'] += 1

    rospy.Subscriber('/voice_words', String, cb, queue_size=10)

    print(u'  ── 请先说唤醒词「老张同学」，再说命令 ──')
    print(u'  监听 %.0f s（Ctrl-C 可提前结束）' % dur)
    t0 = time.time()
    rospy.spin() if dur <= 0 else None
    while time.time() - t0 < dur and not rospy.is_shutdown():
        rospy.sleep(0.2)

    print(u'')
    print(u'  ── 汇总 ──')
    print(u'  识别到 %d 句，唤醒 %d 次，命令 %d 句' % (state['n'], state['wake'], state['cmd']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
