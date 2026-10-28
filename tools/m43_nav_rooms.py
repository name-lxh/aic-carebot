# -*- coding: utf-8 -*-
"""m43_nav_rooms.py —— 只测导航：三房间的 "E点 -> 中心 -> 出门E点" 循环

    python2 m43_nav_rooms.py [BCD]

不扫二维码、不查环境、不做人脸、不做任务、不对话——**只发导航目标**。
用来单独验证：
  1. inflation 0.40 -> 0.30 之后窄门洞能不能过
  2. 门口 E 点作为中转，进出房间是否还卡
"""
from __future__ import print_function

import codecs
import sys
import time

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

    order_txt = sys.argv[1].upper() if len(sys.argv) > 1 else 'BCD'
    order, why = m.parse_order(order_txt)
    if not order:
        print('  [FAIL] 顺序解析失败: %s' % why)
        return 1

    rospy.init_node('nav_rooms', anonymous=True)
    nav = m.Nav(m.Guard(enabled=False))
    if not nav.ready():
        print('  [FAIL] move_base 不可用')
        return 1

    p = nav.pose()
    print('  当前位姿 %s' % ('(%.2f, %.2f, %.1f°)' % p if p else '未知'))
    print(u'  顺序: %s' % u' -> '.join(m.ROOMS[r][2] for r in order))

    ok = 0
    total = 0
    t0 = time.time()
    for i, rid in enumerate(order):
        door, center, cn, task, ep = m.ROOMS[rid]
        cx, cy, _ = m.WP[center]
        ex, ey, _ = m.WP[ep]
        for label, x, y in ((u'%s 门口E点(进)' % cn, ex, ey),
                            (u'%s 房间中心' % cn, cx, cy),
                            (u'%s 门口E点(出)' % cn, ex, ey)):
            total += 1
            st, dt, err = nav.goto(label, x, y, None)
            good = st in ('SUCCEEDED', 'DRY')
            ok += 1 if good else 0
            print(u'  %-18s %-10s %5.1f s  误差 %.3f m' % (label, st, dt, err))
            if not good:
                print(u'     -> 这段失败，接着测下一段')
            time.sleep(0.5)

    print(u'')
    print(u'  导航成功率 %d / %d   总用时 %.1f s' % (ok, total, time.time() - t0))
    return 0 if ok == total else 2


if __name__ == '__main__':
    sys.exit(main())
