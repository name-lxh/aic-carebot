# -*- coding: utf-8 -*-
"""nav_to_a.py —— 从起点导航到 A 点（语音交互前的定位点）"""
from __future__ import print_function
import codecs, sys
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
    rospy.init_node('nav_to_a', anonymous=True)
    nav = m.Nav(m.Guard(enabled=False))
    if not nav.ready():
        print('  [FAIL] move_base 不可用'); return 1
    p = nav.pose()
    print('  当前位姿 %s' % ('(%.2f, %.2f, %.1f)' % p if p else '未知'))
    st, dt, err = nav.goto_named('A')
    print('  -> A 结果=%s 用时 %.1f s 误差 %.3f m' % (st, dt, err))
    return 0 if st in ('SUCCEEDED', 'DRY') else 2

if __name__ == '__main__':
    sys.exit(main())
