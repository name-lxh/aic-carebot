# -*- coding: utf-8 -*-
"""enroll_face.py —— 把一张人脸照片注册进 LBPH trainer.yml（车上跑，python2）

    python2 enroll_face.py <照片> <新id> [trainer.yml]

做三件事：
  1. 用同一份 haarcascade 检出人脸，取最大的一张
  2. 从这张脸生成若干增强样本（镜像/轻微缩放/亮度），避免"一张样本过拟合"
  3. 把样本 **追加**（update）进现有 trainer.yml，先备份原文件
"""
from __future__ import print_function

import os
import shutil
import sys
import time

import cv2
import numpy as np

CASCADE = ('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/'
           'haarcascade_frontalface_default.xml')
TRAINER = ('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/'
           'script/source/trainer/trainer.yml')


def make_variants(gray):
    """从一张脸生成若干变体，让 LBPH 对角度/亮度稍有容忍。"""
    out = [gray]
    out.append(cv2.flip(gray, 1))
    for scale in (0.9, 1.1):
        h, w = gray.shape[:2]
        m = cv2.resize(gray, (int(w * scale), int(h * scale)))
        out.append(m)
    for delta in (15, -15):
        out.append(np.clip(gray.astype(np.int16) + delta, 0, 255).astype(np.uint8))
    return out


def main():
    img_path = sys.argv[1]
    new_id = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    trainer = sys.argv[3] if len(sys.argv) > 3 else TRAINER

    gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print('  [FAIL] 读不到图片: %s' % img_path)
        return 1
    print('  图片: %dx%d' % (gray.shape[1], gray.shape[0]))

    casc = cv2.CascadeClassifier(CASCADE)
    faces = casc.detectMultiScale(gray, 1.3, 5)
    if len(faces) == 0:
        print('  [FAIL] 没检出人脸（换一张正脸照，或调 detectMultiScale 参数）')
        return 1
    x, y, w, h = max(faces, key=lambda r: r[2] * r[3])
    print('  检出 %d 张脸，取最大: (%d,%d) %dx%d' % (len(faces), x, y, w, h))
    crop = gray[y:y + h, x:x + w]

    imgs = make_variants(crop)
    labels = np.array([new_id] * len(imgs))
    print('  生成 %d 个样本，label=%d' % (len(imgs), new_id))

    rec = cv2.face.createLBPHFaceRecognizer()
    if os.path.isfile(trainer):
        bk = trainer + '.bak_' + time.strftime('%Y%m%d_%H%M%S')
        shutil.copy2(trainer, bk)
        print('  已备份 -> %s' % os.path.basename(bk))
        rec.load(trainer)
        try:
            before = rec.getLabels()
            print('  原模型样本数: %s' % (len(before) if before is not None else '?'))
        except Exception:
            pass
        rec.update(imgs, labels)
        print('  已追加到原模型')
    else:
        rec.train(imgs, labels)
        print('  原模型不存在，新建')
    rec.save(trainer)
    print('  已保存: %s' % trainer)

    # 立刻自检：拿原图的那张脸去 predict
    for name, q in (('原图', crop), ('镜像', cv2.flip(crop, 1))):
        label, dist = rec.predict(q)
        print('  自检[%s]: id=%d  distance=%.1f  confidence=%.1f'
              % (name, label, dist, 100.0 - dist))
    return 0


if __name__ == '__main__':
    sys.exit(main())
