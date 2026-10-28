# -*- coding: utf-8 -*-
"""capture_face_dataset.py —— 按相机视角采人脸样本（可迁移的数据集目录）

    python2 capture_face_dataset.py <输出目录> [张数] [总秒数] [label]

默认：100 张 / 300 s（每 3 s 一张）。

## 为什么保存"整帧 + 裁剪"两份

* 整帧（640x480）：以后换车/换相机，可以按新相机的人脸尺寸重新裁、重新对齐尺度
* 裁剪：直接给 train_faces_from_dir.py 用

## 为什么单独存训练集

原来 trainer.yml 放在厂商共用工作空间里
（~/omniWheelCareRobot/.../trainer/trainer.yml），其他组重训一次就把我们的人脸
冲掉了（2026-10-21 实测：昨晚采的 38 张样本全没了）。
所以数据集和模型都放到我们自己的目录，随代码一起迁移。

用法：
    python2 capture_face_dataset.py ~/Hajimi/LXH/face_dataset/raw 100 300 2
"""
from __future__ import print_function

import codecs
import os
import sys
import time

import cv2
import numpy as np
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

CASCADE = ('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/'
           'haarcascade_frontalface_default.xml')
TOPIC = '/camera/rgb/image_raw'


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else '/home/ub/Hajimi/LXH/face_dataset/raw'
    want = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    total = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0

    if not os.path.isdir(outdir):
        os.makedirs(outdir)

    rospy.init_node('capture_face_dataset', anonymous=True)
    say_pub = rospy.Publisher('/voice/say', String, queue_size=5)
    rospy.sleep(0.8)

    def say(text):
        print(u'  [语音] %s' % text)
        try:
            say_pub.publish(String(data=text.encode('utf-8')))
        except Exception:
            pass

    bridge = cv_bridge.CvBridge()
    casc = cv2.CascadeClassifier(CASCADE)

    interval = total / float(want)
    print(u'  目标 %d 张 / %.0f s（每 %.1f s 一张）-> %s' % (want, total, interval, outdir))
    say(u'开始检测')

    t0 = time.time()
    got = 0
    miss = 0
    next_t = t0
    while got < want and not rospy.is_shutdown():
        now = time.time()
        if now < next_t:
            rospy.sleep(min(0.2, next_t - now))
            continue
        next_t += interval
        try:
            msg = rospy.wait_for_message(TOPIC, Image, timeout=3.0)
        except Exception:
            miss += 1
            continue
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        # ★ 检测用直方图均衡后的图（专治逆光：窗户在背后时人脸是暗的），
        #   但**裁剪仍然从原图 gray 取**，保证训练样本和 face_gate 的输入同分布
        det = cv2.equalizeHist(gray)
        faces = casc.detectMultiScale(det, scaleFactor=1.2, minNeighbors=5,
                                      minSize=(int(0.1 * w), int(0.1 * h)))
        if len(faces) == 0:
            faces = casc.detectMultiScale(det, scaleFactor=1.1, minNeighbors=3)
        if len(faces) == 0:
            miss += 1
            continue
        x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
        # 多留 15% 边，避免切掉下巴/额头
        pad_w, pad_h = int(fw * 0.15), int(fh * 0.15)
        x0 = max(0, x - pad_w)
        y0 = max(0, y - pad_h)
        x1 = min(w, x + fw + pad_w)
        y1 = min(h, y + fh + pad_h)
        got += 1
        cv2.imwrite(os.path.join(outdir, 'frame_%03d.jpg' % got), img)
        cv2.imwrite(os.path.join(outdir, 'face_%03d.jpg' % got), gray[y0:y1, x0:x1])
        if got % 10 == 0:
            print(u'    已采 %d 张（漏 %d）  用时 %.0f s  脸 %dx%d'
                  % (got, miss, time.time() - t0, fw, fh))
            say(u'已采%d张' % got)

    print(u'')
    print(u'  完成：%d 张，漏检 %d 次，用时 %.0f s' % (got, miss, time.time() - t0))
    say(u'采集完成，共%d张' % got)
    return 0 if got >= 1 else 1


if __name__ == '__main__':
    sys.exit(main())
