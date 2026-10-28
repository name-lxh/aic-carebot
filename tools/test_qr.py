# -*- coding: utf-8 -*-
"""test_qr.py —— 原地测二维码识别（3 轮，语音报"开始识别"和结果顺序）

    python2 test_qr.py [轮数]

每轮：
  1. 播报「第 N 轮，开始识别」
  2. 调 /qr/detect_once（阻塞，最多 15 s）
  3. 成功 -> 解析顺序并播报「识别成功，顺序是 卧室、厨房、书房」
     失败 -> 播报「第 N 轮识别失败」
  4. 每轮之间停 2 s

不移动底盘，只调用服务 + 播报。
"""
from __future__ import print_function

import codecs
import re
import sys
import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

# 2=B 书房  3=C 厨房  4=D 卧室（现场实测口径，见 m36 注释）
CN = {2: u'书房', 3: u'厨房', 4: u'卧室'}
LEGACY = {1: 2, 2: 3, 3: 4}


def to_text(s):
    try:
        tt = unicode
    except NameError:
        tt = str
    if isinstance(s, tt):
        return s
    return s.decode('utf-8', 'ignore') if isinstance(s, str) else tt(s)


def parse(msg):
    """从服务返回里挖出顺序（2/3/4）。"""
    t = to_text(msg)
    nums = [int(v) for v in re.findall(r'\d', t)]
    if len(nums) == 3:
        if set(nums) == {2, 3, 4}:
            return nums, u'数字'
        if set(nums) == {1, 2, 3}:
            return [LEGACY[v] for v in nums], u'数字(1/2/3 换算)'
    letters = [c.upper() for c in re.findall(r'[BbCcDd]', t)]
    if len(letters) == 3 and set(letters) == {'B', 'C', 'D'}:
        m = {'B': 2, 'C': 3, 'D': 4}
        return [m[c] for c in letters], u'字母'
    return None, u'解析不了: %r' % t


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rospy.init_node('test_qr', anonymous=True)
    say_pub = rospy.Publisher('/voice/say', String, queue_size=5)
    rospy.sleep(1.0)

    def say(text):
        print(u'  [语音] %s' % text)
        try:
            say_pub.publish(String(data=text.encode('utf-8')))
        except Exception as exc:
            print('    (发布失败: %r)' % exc)

    rospy.wait_for_service('/qr/detect_once', timeout=10)
    srv = rospy.ServiceProxy('/qr/detect_once', Trigger)

    results = []
    for r in range(1, rounds + 1):
        print('')
        print(u'  ==== 第 %d 轮 ====' % r)
        say(u'第%d轮，开始识别' % r)
        t0 = time.time()
        try:
            resp = srv()
        except Exception as exc:
            print('  服务异常: %r' % exc)
            say(u'第%d轮服务异常' % r)
            results.append((r, None, u'服务异常'))
            time.sleep(2)
            continue
        dt = time.time() - t0
        ok = bool(resp.success)
        msg = to_text(resp.message)
        print('  success=%s  用时 %.1f s  message=%r' % (ok, dt, msg[:120]))

        if ok:
            order, how = parse(msg)
            if order:
                names = u'、'.join(CN[v] for v in order)
                print(u'  顺序 = %s（%s）' % (names, how))
                say(u'识别成功，顺序是%s' % names)
                results.append((r, order, how))
            else:
                say(u'识别到了，但内容看不懂')
                results.append((r, None, u'内容无法解析'))
        else:
            say(u'第%d轮识别失败' % r)
            results.append((r, None, msg[:40]))
        time.sleep(2)

    print('')
    print('  ==== 汇总 ====')
    good = 0
    for r, order, note in results:
        if order:
            good += 1
            print(u'    第 %d 轮: 成功  %s' % (r, u' -> '.join(CN[v] for v in order)))
        else:
            print(u'    第 %d 轮: 失败  %s' % (r, note))
    print('  成功 %d / %d' % (good, rounds))
    say(u'三轮测试结束，成功%d次' % good)
    return 0 if good == rounds else 1


if __name__ == '__main__':
    sys.exit(main())
