# -*- coding: utf-8 -*-
"""m42_face_only.py —— 只跑正式流程里的"人脸核验"那一段

直接复用 m36_mission_v5 里的：
    Nav / RosDevices / Guard / Mission.phase_face_gate()
所以跑的就是正式流程的同一份代码路径（导航到 A_mid -> A，播报"请正对镜头"，
最多 3 次 /face/verify，成功播"欢迎回家"，失败播"记录后继续巡视"）。

不动房间、不查二维码、不返回起点。
"""
from __future__ import print_function

import codecs
import sys

import rospy

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')


def main():
    import m36_mission_v5 as m

    rospy.init_node('face_only', anonymous=True)
    guard = m.Guard(enabled=False)
    nav = m.Nav(guard)
    if not nav.ready():
        print('  [FAIL] move_base 不可用')
        return 1

    p = nav.pose()
    if p:
        print('  当前位姿 (%.2f, %.2f, %.1f°)' % (p[0], p[1], p[2]))
    else:
        print('  [FAIL] 拿不到位姿（TF 没通）')
        return 1

    dev = m.RosDevices()
    mission = m.Mission(dev, nav)

    print('')
    print('=' * 60)
    print('  只测人脸核验（正式流程同一份代码）')
    print('=' * 60)
    ok = mission.phase_face_gate()

    print('')
    print('=' * 60)
    print('  结果：%s' % ('通过 ✅' if ok else '未通过（但流程会继续）'))
    print('=' * 60)
    for ph, text in mission.timeline:
        print('  %-14s %s' % (ph, text))
    return 0


if __name__ == '__main__':
    sys.exit(main())
