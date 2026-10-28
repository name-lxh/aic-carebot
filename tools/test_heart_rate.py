# -*- coding: utf-8 -*-
"""自测：模拟 ESP32 发一帧心率数据，验证 m36 里的 Vitals 与播报文案。

不依赖 ROS master：只测 UDP 收包 + 播报逻辑（用 shim 对象调 Mission 的方法）。
用法：source env.sh && PYTHONPATH=~/Hajimi/LXH/_tools python2 -u /tmp/test_hr.py
"""
import socket
import threading
import time

import m36_mission_v5 as m

PORT = m.HR_PORT


class Shim(m.Mission):
    """继承 Mission 但绕过 __init__（不需要 ROS），只提供播报用到的属性。"""

    def __init__(self, vitals, dev):
        self.vitals = vitals
        self.dev = self          # 自己实现 say()，省掉一个假设备
        self.said = []
        self.timeline = []

    def log(self, phase, text):
        print(u'  [log] %s %s' % (phase, text))

    def say(self, text):
        self.said.append(text)
        print(u'  [语音] %s' % text)


def sender():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    time.sleep(0.4)
    for pkt in (u'101,72.4,16.5,1,9001', u'102,78.0,17.2,1,9002',
                u'103,88.6,18.0,2,9003'):
        s.sendto(pkt.encode('utf-8'), ('127.0.0.1', PORT))
        print(u'  [sim] 发送 %s' % pkt)
        time.sleep(0.35)


print(u'=== 1. Vitals 收包 ===')
threading.Thread(target=sender).start()
v = m.Vitals()
rec = v.read()
print(u'  收到: %s' % rec)
assert rec is not None, u'没收到包'
assert m.HR_MIN <= rec['heart_rate'] <= m.HR_MAX, u'心率解析不在合理区间'
print(u'  说明：若现场 ESP32 正在广播，这里收到的是真实数据（源 %s）' % rec['source'])

print(u'')
print(u'=== 2. 有数据时的播报 ===')
shim = Shim(v, None)
ok = shim.announce_heart_rate()
print(u'  返回 %s，文案：%s' % (ok, shim.said[-1] if shim.said else u'(无)'))
assert ok and u'心率' in shim.said[-1] and u'次每分' in shim.said[-1], u'播报文案不对'

print(u'')
print(u'=== 3. 没数据时的降级（端口占用 + 无缓存） ===')
v2 = m.Vitals()
v2._last = None
v2._err = 'simulated: port busy'
shim2 = Shim(v2, None)
ok2 = shim2.announce_heart_rate()
print(u'  返回 %s，文案：%s' % (ok2, shim2.said[-1] if shim2.said else u'(无)'))
assert ok2 is False, u'降级返回值不对'

print(u'')
print(u'全部测试通过')
