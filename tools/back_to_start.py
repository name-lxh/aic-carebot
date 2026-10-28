# -*- coding: utf-8 -*-
"""back_to_start.py —— 语音测试结束后从 A 点回起点，并关掉测试中打开的电器。

用法（车在 A 点）：
    export ROS_MASTER_URI=http://192.168.8.213:11399 ROS_IP=192.168.8.213
    python2 -u back_to_start.py
"""
import sys

import rospy

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')


def main():
    import m36_mission_v5 as m

    rospy.init_node('back_to_start', anonymous=True)
    nav = m.Nav(m.Guard(enabled=False))
    if not nav.ready():
        print('  [FAIL] move_base 不可用')
        return 1

    p = nav.pose()
    print('  当前位姿 %s' % ('(%.2f, %.2f, %.1f)' % p if p else '未知'))

    st, dt, err = nav.goto_named('living_in')
    print('  -> 客厅 结果=%s 用时 %.1f s 误差 %.3f m' % (st, dt, err))

    st, dt, err = nav.goto(u'返回起点', 0.0, 0.0, None)
    print('  -> 起点 结果=%s 用时 %.1f s 误差 %.3f m' % (st, dt, err))

    # 关掉测试里可能被打开的电器
    try:
        dev = m.RosDevices()
        for name, act in (('fan_kitchen', 'off'), ('light_bedroom', 'off')):
            ok, msg = dev.iot_control(name, act)
            print('  %s %s -> %s' % (name, act, ok))
    except Exception as exc:
        print('  [warn] 关设备失败：%r' % exc)

    print('  回到起点，语音测试流程结束')
    return 0 if st in ('SUCCEEDED', 'DRY') else 2


if __name__ == '__main__':
    sys.exit(main())
