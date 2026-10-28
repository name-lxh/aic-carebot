# -*- coding: utf-8 -*-
"""voice_probe.py —— 语音识别自检：等一句话，打印识别文本与意图匹配结果。

用法（车上，先起好 master + ASR）：
    PYTHONPATH=~/Hajimi/LXH/_tools python2 -u voice_probe.py [--timeout 12]
退出码：0 = 识别到；1 = 超时；2 = 识别到但意图匹配失败
"""
from __future__ import print_function

import sys
import time

import rospy
from std_msgs.msg import String

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')

ASK = u'查询室内温度'


def main():
    timeout = 12.0
    if '--timeout' in sys.argv:
        timeout = float(sys.argv[sys.argv.index('--timeout') + 1])

    rospy.init_node('voice_probe', anonymous=True)
    got = [None]

    def cb(msg):
        try:
            text = msg.data.decode('utf-8') if isinstance(msg.data, str) else msg.data
        except Exception:
            text = u''
        text = (text or u'').strip()
        if text:
            got[0] = text

    rospy.Subscriber('/voice_words', String, cb, queue_size=10)
    rospy.sleep(0.5)
    print(u'  请说：%s（%.0f 秒内）' % (ASK, timeout))
    t0 = time.time()
    while time.time() - t0 < timeout and got[0] is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if not got[0]:
        print(u'  [FAIL] %.0f 秒内没有识别结果' % timeout)
        return 1

    print(u'  识别到：[%s]' % got[0])
    try:
        from intent_match import match
        r = match(got[0])
        if not r:
            print(u'  [WARN] 识别成功，但意图匹配失败（低于阈值 0.80）')
            return 2
        print(u'  意图：%s（命中"%s" %.2f）' % r)
    except Exception as exc:
        print(u'  [WARN] 意图匹配模块异常：%r' % exc)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
