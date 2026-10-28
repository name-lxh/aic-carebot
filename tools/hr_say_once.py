# -*- coding: utf-8 -*-
"""一次性心率播报：读一帧心率 -> 通过 /voice/say 播一遍 -> 退出。

用法（需要一个 ROS master + 正在跑的 tts_net）：
    export ROS_MASTER_URI=http://192.168.8.213:11399
    PYTHONPATH=~/Hajimi/LXH/_tools python2 -u hr_say_once.py
"""
import sys

import rospy
from std_msgs.msg import String

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')
import m36_mission_v5 as m


def main():
    rospy.init_node('hr_say_once', anonymous=True)
    pub = rospy.Publisher('/voice/say', String, queue_size=5)
    rospy.sleep(1.0)

    v = m.Vitals().read()
    if v:
        hr = int(round(v['heart_rate']))
        text = u'您当前的心率是 %d 次每分' % hr
        print(u'[数据] 心率 %.1f，呼吸 %.1f，人数 %d，源 %s'
              % (v['heart_rate'], v.get('respiration', 0), v.get('people', 0),
                 v.get('source', '?')))
    else:
        text = u'暂时没有收到心率数据'
        print(u'[数据] 没收到心率数据（UDP %d）' % m.HR_PORT)

    print(u'[播报] %s' % text)
    msg = String()
    msg.data = text.encode('utf-8')
    pub.publish(msg)
    rospy.sleep(1.0)
    print(u'[完成] 已发出播报')
    return 0 if v else 3


if __name__ == '__main__':
    sys.exit(main())
