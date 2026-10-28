# -*- coding: utf-8 -*-
"""enroll_live.py —— 用**实时相机画面**注册人脸（解决"证件照训练、现场认不出"）

    python2 enroll_live.py <id> [帧数]

## 为什么必须用实时画面

`face_gate` 的识别是这样做的：

    faces = cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(0.1w,0.1h))
    fid, dist = recognizer.predict(gray[y:y+h, x:x+w])      # ★ 原始裁剪，没有 resize

而 2026-10-20 我用**证件照**注册的那次，裁剪出来是 **654×654**；
现场相机 640×480 里正常距离的人脸只有 **100~200 px**。
LBPH 是在原始像素上算局部二值模式的，尺度差 4~6 倍 → 直方图完全对不上
→ 现场 26/26 帧都检出了人脸，但"命中主人 0 轮"。

所以要用同一台相机、同一距离采样本。

## 做法

连续抓 N 帧，每帧检出最大人脸，把这 N 个裁剪 **追加**（update）到指定 id。
"""
from __future__ import print_function

import os
import shutil
import sys
import time

# ---- 输出编码兜底（重定向时 locale 可能是 ASCII）----
import codecs as _codecs
for _n in ('stdout', 'stderr'):
    _st = getattr(sys, _n, None)
    _e = (getattr(_st, 'encoding', None) or '').lower()
    if _st is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, _codecs.getwriter('utf-8')(_st))
        except Exception:
            pass

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

_SAY = None


def say(text):
    try:
        print(u'  [voice] %s' % text)
    except Exception:
        try:
            print('  [voice] (encoding issue)')
        except Exception:
            pass
    if _SAY is not None:
        try:
            _SAY.publish(String(data=text.encode('utf-8')))
        except Exception as exc:
            print('  (publish failed: %r)' % exc)

CASCADE = ('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/'
           'haarcascade_frontalface_default.xml')
TRAINER = ('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/'
           'script/source/trainer/trainer.yml')
TOPIC = '/camera/rgb/image_raw'


def main():
    face_id = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    want = int(sys.argv[2]) if len(sys.argv) > 2 else 12

    global _SAY
    rospy.init_node('enroll_live', anonymous=True)
    _SAY = rospy.Publisher('/voice/say', String, queue_size=5)
    rospy.sleep(1.0)
    bridge = cv_bridge.CvBridge()
    casc = cv2.CascadeClassifier(CASCADE)

    rec = cv2.face.createLBPHFaceRecognizer()
    rec.load(TRAINER)
    _lab = rec.getLabels()
    _n = 0 if _lab is None else len(_lab)
    print('  已加载 trainer（原有 %d 样本）' % _n)

    say(u'正在采集，请正对摄像头，距离半米到一米')
    print('  请正对摄像头、距离 0.6~1.2 m，保持 %d 秒……' % (want // 2))
    samples, sizes = [], []
    t_end = time.time() + max(30.0, want / 2.0 + 15.0)
    while len(samples) < want and time.time() < t_end and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message(TOPIC, Image, timeout=3.0)
        except Exception:
            continue
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        faces = casc.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5,
                                      minSize=(int(0.1 * w), int(0.1 * h)))
        if len(faces) == 0:
            continue
        x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
        crop = gray[y:y + fh, x:x + fw]
        samples.append(crop)
        sizes.append((fw, fh))
        print('    采到第 %d 个样本: %dx%d' % (len(samples), fw, fh))
        if len(samples) in (1, 5, 10):
            say(u'第 %d 张，很好，继续' % len(samples))
        time.sleep(0.15)

    if not samples:
        say(u'没有采集到人脸，请靠近一点再试一次')
        print('  [FAIL] 一个样本都没采到（没人脸 / 相机没数据）')
        return 1

    bk = TRAINER + '.bak_live_' + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(TRAINER, bk)
    print('  已备份 -> %s' % os.path.basename(bk))

    rec.update(samples, np.array([face_id] * len(samples), dtype=np.int32))
    rec.save(TRAINER)
    print('  已追加 %d 个样本（id=%d，尺寸 %dx%d ~ %dx%d）并保存'
          % (len(samples), face_id, min(s[0] for s in sizes), min(s[1] for s in sizes),
             max(s[0] for s in sizes), max(s[1] for s in sizes)))

    ok = 0
    for i, s in enumerate(samples[:min(len(samples), 6)]):
        label, dist = rec.predict(s)
        hit = (label == face_id)
        ok += 1 if hit else 0
        print('    自检 %d: id=%d dist=%.1f conf=%.1f %s'
              % (i + 1, label, dist, 100.0 - dist, '命中' if hit else '未命中'))
    say(u'采集完成，共 %d 张' % len(samples))
    print('  ✅ 完成（%d/%d 自检命中）' % (ok, min(len(samples), 6)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
