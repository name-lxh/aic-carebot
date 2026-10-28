# -*- coding: utf-8 -*-
"""
test_drive.py —— 极短受控测试（第一次真机运动）

## 测什么

  1. **IMU 转向闭环准不准** —— 原地转 +90°，看实际转了多少
  2. **轮式里程计在"运动时"到底能不能用** —— 同时记录 IMU 和 /odom 的 yaw，
     对比两者。静止时 /odom 漂 44°，但**运动时**也许可用（需实测）
  3. **激光急停灵不灵** —— 前进 0.4 m，全程盯前方
  4. **底盘响应** —— 加速度斜坡、停稳时间

## 安全

  · 速度 0.12 m/s（很慢）
  · 前方 < 0.40 m 立即停
  · 每步硬超时
  · 看门狗兜底（主线程卡死也会压 0）
  · **测试前先看前方 1 m 内是否空**

⚠️ 跑之前确认：场地里没人、机器人周围 0.5 m 内无物。
"""
from __future__ import print_function

import math
import sys
import time

import rospy

sys.path.insert(0, '/tmp')
from realbot import RealBot, quat_to_yaw, norm_angle   # noqa: E402


def main():
    rospy.init_node('test_drive', anonymous=True)
    d = RealBot(dry_run=('--dry' in sys.argv))

    print('')
    print('=' * 62)
    print('  真机受控测试：前进 0.4 m + 原地转 90°')
    print('=' * 62)

    if not d.ready(timeout=20):
        print('  ✘ /scan 或 /imu_data 收不到，中止')
        return 1

    # ---- 出发前检查 ----
    print('')
    print('---- 出发前检查 ----')
    yaw0 = d.yaw()
    print('  IMU yaw      = %.2f°' % math.degrees(yaw0))
    wp = d.wheel_pose()
    if wp:
        print('  轮式里程计    = x %.2f  y %.2f  yaw %.2f°（不可信，仅记录）'
              % (wp[0], wp[1], math.degrees(wp[2])))
    fm = d.sector_min(0.0)
    print('  前方 ±40° 最近 = %s'
          % (('%.2f m' % fm) if fm is not None else '无有效点'))
    for a, nm in ((90, '左侧'), (270, '右侧'), (180, '后方')):
        v = d.sector_min(math.radians(a))
        print('  %s(%.0f°) 最近 = %s'
              % (nm, a, ('%.2f m' % v) if v is not None else '无有效点'))

    if fm is not None and fm < 0.60:
        print('')
        print('  ★ 前方只有 %.2f m —— 太近，拒绝开始测试。' % fm)
        print('    请把机器人转向更空的方向，或先挪开障碍。')
        return 2

    print('')
    print('  ⚠️ 3 秒后开始移动，请确认场地里没人')
    for i in (3, 2, 1):
        print('     %d...' % i)
        time.sleep(1.0)

    # ---- 步骤 1：前进 0.4 m ----
    print('')
    print('---- 步骤 1: 前进 0.40 m ----')
    t0 = time.time()
    y_before = d.yaw()
    o_before = d.wheel_pose()
    ok1 = d.forward(0.40)
    time.sleep(0.8)
    y_after = d.yaw()
    o_after = d.wheel_pose()

    print('  IMU  yaw: %.2f° -> %.2f°  变化 %+.2f°'
          % (math.degrees(y_before), math.degrees(y_after),
             math.degrees(norm_angle(y_after - y_before))))
    if o_before and o_after:
        print('  轮式 odom: (%.2f, %.2f) -> (%.2f, %.2f)'
              % (o_before[0], o_before[1], o_after[0], o_after[1]))
        print('       位移 = %.3f m   （这个是 STM32 报的**绝对值差**，'
              '不可信，仅记录）'
              % math.hypot(o_after[0] - o_before[0],
                           o_after[1] - o_before[1]))
    print('  前进是否正常完成: %s' % ('是' if ok1 else '否（被护栏拦停）'))

    # ---- 步骤 2：原地转 +90° ----
    print('')
    print('---- 步骤 2: 原地左转 +90° ----')
    o_before = d.wheel_pose()
    ok2 = d.rotate(90.0)
    time.sleep(0.8)
    o_after = d.wheel_pose()

    print('  IMU 给出的转角就是上面那行 "实际"')
    if o_before and o_after:
        print('  轮式 odom yaw: %.2f° -> %.2f°  变化 %+.2f°'
              % (math.degrees(o_before[2]), math.degrees(o_after[2]),
                 math.degrees(norm_angle(o_after[2] - o_before[2]))))
        print('  ★ 对比 IMU 的 90°：若 odom 差很多，说明运动时也不可用')
    print('  转向是否达标(±8°): %s' % ('是' if ok2 else '否'))

    # ---- 结束 ----
    d.stop('测试结束')
    print('')
    print('=' * 62)
    print('  测试完成，机器人已停止')
    print('  trace: %s' % d.trace)
    print('=' * 62)
    return 0


if __name__ == '__main__':
    sys.exit(main())
