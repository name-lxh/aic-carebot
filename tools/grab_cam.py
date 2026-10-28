# -*- coding: utf-8 -*-
"""grab_cam.py —— 抓一张相机图存成 jpg，顺便用 cascade 看有没有脸（车上跑）"""
from __future__ import print_function

import sys

import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image

out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/cam.jpg'
topic = sys.argv[2] if len(sys.argv) > 2 else '/camera/rgb/image_raw'

rospy.init_node('grab_cam', anonymous=True)
bridge = cv_bridge.CvBridge()
msg = rospy.wait_for_message(topic, Image, timeout=15)
img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
cv2.imwrite(out, img)
print('saved %s  %dx%d' % (out, img.shape[1], img.shape[0]))

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
casc = cv2.CascadeClassifier('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/'
                             'script/haarcascade_frontalface_default.xml')
faces = casc.detectMultiScale(gray, 1.3, 5)
print('faces: %d' % len(faces))

TRAINER = ('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/'
           'script/source/trainer/trainer.yml')
rec = cv2.face.createLBPHFaceRecognizer()
rec.load(TRAINER)
for (x, y, w, h) in faces:
    label, dist = rec.predict(gray[y:y + h, x:x + w])
    print('  face (%d,%d) %dx%d -> id=%d distance=%.1f confidence=%.1f'
          % (x, y, w, h, label, dist, 100.0 - dist))
