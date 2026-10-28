#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""fall_check.py —— 跌倒检测链路自检（不动车）。

做四件事：
  1) 检查 /camera/rgb/image_raw 有没有画面（没有就尝试起 astra）
  2) 起 YOLOv5 worker，打印模型就绪信息与首帧推理耗时
  3) 连续观测 N 秒，打印每一帧的判定状态（WAIT_*/CONFIRM*/FALL_CONFIRMED）
  4) 汇总：看到多少帧、最高置信度、是否确认跌倒

用法：
    python2 fall_check.py                 # 观测 12 秒
    python2 fall_check.py --seconds 20
    python2 fall_check.py --no-camera     # 不自动起相机
"""
from __future__ import print_function

import argparse
import os
import sys
import time

import rospy

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fall_detect  # noqa: E402


def camera_running():
    out = os.popen('timeout 5 rostopic list 2>/dev/null').read()
    return '/camera/rgb/image_raw' in out


def ensure_camera(start=True, timeout=30.0):
    if camera_running():
        print(u'  ✅ 相机已在发布 /camera/rgb/image_raw')
        return True
    if not start:
        print(u'  ❌ 相机没在发布（--no-camera 不自动起）')
        return False
    print(u'  … 相机没起，尝试启动 astra（最多等 %.0f s）' % timeout)
    os.system('setsid nohup roslaunch astra_launch astra.launch '
              '> /tmp/f_astra.log 2>&1 < /dev/null &')
    t0 = time.time()
    while time.time() - t0 < timeout:
        if camera_running():
            print(u'  ✅ 相机已启动（%.0f s）' % (time.time() - t0))
            return True
        time.sleep(1.0)
    print(u'  ❌ 相机启动失败，看 /tmp/f_astra.log')
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--no-camera', action='store_true')
    args = ap.parse_args()

    rospy.init_node('fall_check', anonymous=True)
    print(u'════ 跌倒检测自检 ════')

    if not ensure_camera(start=not args.no_camera):
        return 2

    from factory_fall_logic import FactoryFallEvidence

    cam = fall_detect.LatestImage()
    if cam.get(timeout=5.0) is None:
        print(u'  ❌ 5 s 内没收到相机帧')
        cam.close()
        return 2
    print(u'  ✅ 已收到相机帧')

    det = fall_detect.FallDetector()
    t0 = time.time()
    try:
        det.start()
    except Exception as exc:
        print(u'  ❌ 模型没起来：%r' % exc)
        cam.close()
        return 2
    print(u'  ✅ 模型就绪（%.1f s）' % (time.time() - t0))

    evidence = FactoryFallEvidence()
    frames = 0
    best = 0.0
    confirmed = False
    first_ms = None
    deadline = time.time() + args.seconds
    while time.time() < deadline and not rospy.is_shutdown():
        img = cam.get(timeout=1.0)
        if img is None:
            continue
        t1 = time.time()
        try:
            people = det.observe(img)
        except Exception as exc:
            print(u'  取帧失败：%r' % exc)
            time.sleep(0.2)
            continue
        cost = time.time() - t1
        if first_ms is None:
            first_ms = cost
            print(u'  首帧推理 %.0f ms' % (cost * 1000))
        frames += 1
        for person in people:
            best = max(best, float(person.get('score') or 0.0))
        state = evidence.update(time.time(), people)
        labels = u'、'.join(u'%s(%.2f)' % (p.get('label'), p.get('score') or 0)
                            for p in people) or u'无人'
        print(u'  第 %2d 帧 %5.0f ms  %s  -> %s' % (frames, cost * 1000, labels, state))
        if state == 'FALL_CONFIRMED':
            confirmed = True
            break
        time.sleep(0.15)

    det.close()
    cam.close()
    print(u'')
    print(u'  汇总：%d 帧 / %.1f s，最高置信度 %.2f，%s'
          % (frames, args.seconds, best, u'确认跌倒 ✅' if confirmed else u'未确认跌倒'))
    return 0 if frames > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
