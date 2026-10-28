# -*- coding: utf-8 -*-
"""
calib_turn.py —— 转向标定（只原地旋转，零碰撞风险）

## 为什么要标定

实测：发 15° 的转向指令，IMU 实测只转了 12.4~13.8°（**偏小约 1.2~2.6°**）。
24 步累计差了约 42°。说明底盘对 angular.z 的响应**增益偏小**，
或者存在稳态误差（速度太低时电机克服不了静摩擦）。

## 标定方法

  1. 正向、反向各测若干角度，算出**实测/指令**的比例 k
  2. 用 k 做补偿：指令角 = 期望角 / k
  3. 验证：让它转 90° / 180° / -90°，看误差是否在 ±3° 内

⚠️ 标定只旋转，不改变机器人与墙的距离 —— 不可能撞。
⚠️ 但如果**当前朝向后方很近有墙**，旋转时车体会扫到 —— 实测背后 0.12m，
   转 360° 时理论上车体画一个半径 0.18m 的圆，会**贴到**那 0.12m 的东西。
   所以标定前先确认最小距离 > 0.25m，否则只做小角度摆动。
"""
from __future__ import print_function

import math
import sys
import time

import rospy

sys.path.insert(0, '/tmp')
from realbot import RealBot, norm_angle   # noqa: E402


def measure_turn(d, cmd_deg, settle=0.7):
    """发一个转向指令，返回 (实测角度, 耗时)。"""
    y0 = d.yaw()
    t0 = time.time()
    d.rotate(cmd_deg, timeout=max(20.0, abs(cmd_deg) / 12.0 + 12.0),
             tol_deg=1.5)
    time.sleep(settle)
    y1 = d.yaw()
    return math.degrees(norm_angle(y1 - y0)), time.time() - t0


def main():
    rospy.init_node('calib_turn', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv))

    print('')
    print('=' * 64)
    print('  转向标定（只旋转，不前进）')
    print('=' * 64)

    if not d.ready(timeout=20):
        print('  ✘ 收不到 /scan 或 /imu_data')
        return 1

    # ---- 安全检查：车体旋转会不会碰到东西 ----
    print('')
    print('---- 旋转安全余量检查 ----')
    worst = None
    for a in range(0, 360, 15):
        v = d.sector_min(math.radians(a), half_deg=8.0)
        if v is not None and (worst is None or v < worst[1]):
            worst = (a, v)
    print('  最近障碍：%.3f m（方位 %d°）' % (worst[1], worst[0]))
    R_BODY = 0.18
    if worst[1] < R_BODY + 0.03:
        print('')
        print('  ⚠️★ 最近障碍 %.3f m，只比车体半径 %.2f m 多一点点。'
              % (worst[1], R_BODY))
        print('     旋转时车体边缘可能擦到。**只做小角度摆动**（±20°）。')
        SMALL_ONLY = True
    else:
        print('  ✔ 余量够（%.2f m > 车体 %.2f m），可以整圈旋转'
              % (worst[1], R_BODY))
        SMALL_ONLY = False

    # ---- 标定 ----
    if SMALL_ONLY:
        tests = [20.0, -20.0, 20.0, -20.0]
    else:
        tests = [60.0, -60.0, 60.0, -60.0, 120.0, -120.0]

    print('')
    print('---- 正向/反向测量（指令 → 实测）----')
    res = []
    for c in tests:
        if rospy.is_shutdown():
            break
        got, el = measure_turn(d, c)
        k = got / c if c else 0
        res.append((c, got, k))
        print('  指令 %+7.1f°  →  实测 %+7.1f°   比值 %.3f   用时 %.1f s'
              % (c, got, k, el))
        time.sleep(0.5)

    if not res:
        print('  ✘ 没测到数据')
        return 1

    ks = [k for _c, _g, k in res]
    k_avg = sum(ks) / len(ks)
    print('')
    print('  平均比值 k = %.3f   （<1 表示实际转得比指令少）' % k_avg)

    # ---- 验证补偿 ----
    print('')
    print('---- 用补偿系数验证：目标 90° / -90° / 180° ----')
    targets = [90.0, -90.0, 180.0] if not SMALL_ONLY else [20.0, -20.0]
    ok_n = 0
    for t in targets:
        if rospy.is_shutdown():
            break
        # 补偿：指令 = 目标 / k；再加一点下限避免太慢
        cmd = t / k_avg if abs(k_avg) > 0.3 else t
        cmd = max(-170.0, min(170.0, cmd))
        got, el = measure_turn(d, cmd, settle=1.0)
        err = t - got
        mark = '✔' if abs(err) <= 3.0 else ('·' if abs(err) <= 6.0 else '✘')
        if abs(err) <= 3.0:
            ok_n += 1
        print('  %s 目标 %+7.1f°  补偿后指令 %+7.1f°  →  实测 %+7.1f°'
              '   误差 %+5.1f°  （%.1f s）'
              % (mark, t, cmd, got, err, el))
        time.sleep(0.6)

    print('')
    print('  达标(±3°)：%d / %d' % (ok_n, len(targets)))

    # ---- 保存标定结果 ----
    out = {'k': k_avg, 'samples': res,
           'small_only': SMALL_ONLY,
           'note': 'k = 实测/指令；控制器里用 cmd = want / k 补偿'}
    try:
        import json
        with open('/tmp/turn_calib.json', 'w') as fp:
            json.dump(out, fp, indent=1)
        print('  已存 /tmp/turn_calib.json')
    except Exception as e:
        print('  存盘失败: %r' % e)

    d.stop('标定结束')
    print('=' * 64)
    print('  建议：把 k=%.3f 写进 realbot.py 的 TURN_GAIN' % k_avg)
    print('=' * 64)
    return 0


if __name__ == '__main__':
    sys.exit(main())
