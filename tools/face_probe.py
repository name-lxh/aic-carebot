# -*- coding: utf-8 -*-
"""face_probe.py —— 现场人脸匹配诊断：抓一帧，检出人脸，打印 predict 的 id 和距离"""
from __future__ import print_function

import codecs
import sys

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

CASCADE = '/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/haarcascade_frontalface_default.xml'
TRAINER = '/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/source/trainer/trainer.yml'

rospy.init_node('face_probe', anonymous=True)
bridge = cv_bridge.CvBridge()
casc = cv2.CascadeClassifier(CASCADE)
rec = cv2.face.createLBPHFaceRecognizer()
rec.load(TRAINER)
labels = rec.getLabels()
print('  模型样本 %d 个，label 集合 %s'
      % (len(labels) if labels is not None else 0,
         sorted(set(int(v) for v in labels)) if labels is not None else []))

for k in range(5):
    try:
        msg = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=6.0)
    except Exception:
        print('  [warn] 没拿到图像')
        continue
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    faces = casc.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5,
                                  minSize=(int(0.1 * w), int(0.1 * h)))
    print('  第 %d 帧: 检出 %d 张脸' % (k + 1, len(faces)))
    for (x, y, fw, fh) in faces[:3]:
        label, dist = rec.predict(gray[y:y + fh, x:x + fw])
        print('     %dx%d 位置(%d,%d) -> id=%d  distance=%.1f  confidence=%.1f'
              % (fw, fh, x, y, label, dist, 100.0 - dist))
    rospy.sleep(0.6)
