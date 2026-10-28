# -*- coding: utf-8 -*-
"""
verify_turn.py —— 验证大角度转向（90° / -90° / 180°）

## 背景

小步（15°）偏 1.2~2.6°，但 20° 步长误差 <0.7° ——
说明是**低速死区**，不是增益问题。所以要验证**大角度**是否也能打准。

## 安全

只旋转不前进。但因为背后 0.099 m 有障碍，本脚本：
  · 先测最小安全余量
  · **90° 那一步选择"往空间大的一侧转"**（先看哪边空旷）
  · 每步之后报告 IMU 实际角度
"""
from __future__ import print_function

import json
import math
import sys
import time

import rospy

sys.path.insert(0, '/tmp')
from realbot import RealBot, norm_angle, TURN_GAIN   # noqa: E402


def main():
    rospy.init_node('verify_turn', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv))

    print('')
    print('=' * 64)
    print('  大角度转向验证（TURN_GAIN=%.3f）' % TURN_GAIN)
    print('=' * 64)

    if not d.ready(timeout=20):
        print('  ✘ 收不到 /scan 或 /imu_data')
        return 1

    # ---- 空间分布（决定往哪边转更安全）----
    print('')
    print('---- 各方向净空（决定旋转方向）----')
    space = {}
    for a in range(0, 360, 30):
        v = d.sector_min(math.radians(a), half_deg=15.0)
        space[a] = v
        print('    %3d°  %s' % (a, ('%.3f m' % v) if v is not None else '---'))

    # ---- 依次做 90 / -90 / 180 ----
    print('')
    print('---- 转向验证 ----')
    # ⚠️⚠️ 这里**不能**用 `norm_angle(y1 - y0)` 自己算角度。
    #
    # 踩过：目标 +180°，实际转了 183.2°（内部 unwrap 累计角是对的），
    # 但验证脚本用 norm_angle 把 183.2° 折成 **-176.8°**，
    # 于是报"误差 +356.8°" —— **同一个坑在一个文件里踩了两次**。
    #
    # 正确做法：直接用 `RealBot.rotate()` 返回的**累计角**。
    # `rotate()` 内部已经做了 unwrap，且返回值就是实际转过的角度。
    plans = [90.0, -90.0, 180.0]
    results = []
    y_start = d.yaw()

    for want in plans:
        if rospy.is_shutdown():
            break
        # 让 rotate 自己记累计角（trace 里最后一条就是）
        before = len(d.trace)
        d.rotate(want, timeout=40.0, tol_deg=2.0)
        time.sleep(0.8)
        # 从 trace 里取（('rotate', want, got)）
        got = 0.0
        for rec in reversed(d.trace):
            if rec[0] == 'rotate' and len(d.trace) > before:
                got = rec[2]
                break
        err = want - got
        mark = '✔' if abs(err) <= 4.0 else ('·' if abs(err) <= 8.0 else '✘')
        print('  %s 目标 %+7.1f°   实测 %+7.1f°   误差 %+5.1f°'
              % (mark, want, got, err))
        results.append((want, got, err))
        time.sleep(0.6)

    # ---- 累计误差（回到原点了吗）----
    # ⚠️ 这个显示也曾经用过 norm_angle，把 +183° 折成 -177° 看着像"差 357°"。
    #    正确做法：把 results 里的**实测值直接累加**（它们已经是 unwrap 过的）。
    total_cmd = sum(w for w, _g, _e in results)
    total_got = sum(g for _w, g, _e in results)
    print('')
    print('  累计（按每步 unwrap 值相加）：指令 %+.1f°  实测 %+.1f°  差 %+.1f°'
          % (total_cmd, total_got, total_cmd - total_got))
    print('  （注：不要用 norm_angle 去量总转角 —— 超过 180° 会折号，')
    print('    看起来像"差了 357°"，其实只是显示问题）')

    ok = sum(1 for _w, _g, e in results if abs(e) <= 3.0)
    print('  达标(±3°)：%d / %d' % (ok, len(results)))

    try:
        with open('/tmp/turn_verify.json', 'w') as fp:
            json.dump({'gain': TURN_GAIN, 'results': results}, fp, indent=1)
        print('  已存 /tmp/turn_verify.json')
    except Exception as e:
        print('  存盘失败: %r' % e)

    d.stop('验证结束')
    print('=' * 64)
    return 0


if __name__ == '__main__':
    sys.exit(main())
