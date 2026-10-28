# -*- coding: utf-8 -*-
"""train_faces_from_dir.py —— 用一批多角度照片训练人脸（追加到 trainer.yml）

    python2 train_faces_from_dir.py <图片目录> [label] [目标尺寸]

## 两个关键点

1. **尺度必须对齐相机**。face_gate 是拿相机的原始裁剪直接 predict 的
   （`gray[y:y+h, x:x+w]`，不 resize）。640x480 画面里、0.6~1.2 m 处的人脸
   大约 110~125 px；如果拿手机原图（脸有好几百 px）去训练，LBPH 的直方图
   尺度完全对不上 —— 2026-10-20 用证件照 654x654 训练就是这样翻车的。
   所以这里把每张脸统一缩到 TARGET（默认 115）。

2. **先做留一验证再落盘**。训练前先把"留一张出来"的距离分布打出来，
   能预估现场会不会误拒；如果距离普遍 > 70，说明这批照片和相机差异太大。

流程：
  1. 逐张检测人脸（取最大的一张）-> 缩放到 TARGET
  2. 留一法评估：用其余照片训练临时模型，预测留下的那张，打印距离
  3. 全部样本（含增强）追加到 trainer.yml，先备份原文件
  4. 自检：逐样本 predict
"""
from __future__ import print_function

import codecs
import glob
import os
import shutil
import sys
import time

import cv2
import numpy as np

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
TRAINER = ('/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/'
           'script/source/trainer/trainer.yml')


def detect_face(gray, casc):
    h, w = gray.shape[:2]
    faces = casc.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=4,
                                  minSize=(int(0.08 * w), int(0.08 * h)))
    if len(faces) == 0:
        faces = casc.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)
    if len(faces) == 0:
        return None
    return max(faces, key=lambda r: r[2] * r[3])


def variants(face, target):
    """统一缩放到 target，再做轻量增强（镜像/微转/缩放/亮度）。"""
    base = cv2.resize(face, (target, target), interpolation=cv2.INTER_AREA)
    out = [base]
    out.append(cv2.flip(base, 1))
    for ang in (-8, 8):
        m = cv2.getRotationMatrix2D((target / 2.0, target / 2.0), ang, 1.0)
        out.append(cv2.warpAffine(base, m, (target, target),
                                  flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_REPLICATE))
    for sc in (0.92, 1.08):
        side = int(target * sc)
        tmp = cv2.resize(base, (side, side), interpolation=cv2.INTER_AREA)
        if side >= target:
            tmp = tmp[:target, :target]
        else:
            pad = (target - side) // 2
            tmp = cv2.copyMakeBorder(tmp, pad, target - side - pad,
                                     pad, target - side - pad,
                                     cv2.BORDER_REPLICATE)
        out.append(tmp)
    for d in (12, -12):
        out.append(np.clip(base.astype(np.int16) + d, 0, 255).astype(np.uint8))
    return out


def main():
    src = sys.argv[1]
    label = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    target = int(sys.argv[3]) if len(sys.argv) > 3 else 115

    files = []
    for ext in ('*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG'):
        files.extend(glob.glob(os.path.join(src, ext)))
    files = sorted(set(files))
    if not files:
        print('  [FAIL] 目录里没有图片: %s' % src)
        return 1
    print('  找到 %d 张图片，目标尺寸 %d px，label=%d' % (len(files), target, label))

    casc = cv2.CascadeClassifier(CASCADE)
    faces, used = [], []
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print('    %-40s 读不到' % os.path.basename(f))
            continue
        box = detect_face(img, casc)
        if box is None:
            print('    %-40s 没检出人脸' % os.path.basename(f))
            continue
        x, y, w, h = box
        base = cv2.resize(img[y:y + h, x:x + w], (target, target),
                          interpolation=cv2.INTER_AREA)
        faces.append(base)
        used.append(os.path.basename(f))
        print('    %-40s 脸 %dx%d -> %dpx' % (os.path.basename(f), w, h, target))

    if len(faces) < 3:
        print('  [FAIL] 可用人脸不足 3 张，没法做留一验证')
        return 1

    # ---- 留一验证 ----
    print('')
    print('  ── 留一验证（用其余照片训练，预测留下的那张）──')
    dists = []
    for i in range(len(faces)):
        tr = [faces[j] for j in range(len(faces)) if j != i]
        rec = cv2.face.createLBPHFaceRecognizer()
        rec.train(tr, np.array([label] * len(tr), dtype=np.int32))
        _, dist = rec.predict(faces[i])
        dists.append(dist)
    dists_sorted = sorted(dists)
    print('    距离: %s' % ' '.join('%.1f' % d for d in dists))
    print('    最小 %.1f  中位 %.1f  最大 %.1f  （置信度 = 100-距离）'
          % (dists_sorted[0], dists_sorted[len(dists_sorted) // 2], dists_sorted[-1]))

    # ---- 落盘 ----
    samples, labels = [], []
    for f in faces:
        for v in variants(f, target):
            samples.append(v)
            labels.append(label)
    print('')
    print('  生成 %d 个训练样本（每张 %d 个变体）'
          % (len(samples), len(samples) // len(faces)))

    if os.path.isfile(TRAINER):
        bk = TRAINER + '.bak_dir_' + time.strftime('%Y%m%d_%H%M%S')
        shutil.copy2(TRAINER, bk)
        print('  已备份 -> %s' % os.path.basename(bk))
        rec = cv2.face.createLBPHFaceRecognizer()
        rec.load(TRAINER)
        old = rec.getLabels()
        print('  原模型样本 %d 个' % (len(old) if old is not None else 0))
        rec.update(samples, np.array(labels, dtype=np.int32))
        print('  已追加')
    else:
        rec = cv2.face.createLBPHFaceRecognizer()
        rec.train(samples, np.array(labels, dtype=np.int32))
        print('  新建模型')
    rec.save(TRAINER)

    lab = rec.getLabels()
    print('  保存完成：%s  共 %d 样本，label=%s'
          % (TRAINER, len(lab) if lab is not None else 0,
             sorted(set(int(v) for v in lab)) if lab is not None else []))

    print('')
    print('  ── 自检（同一批样本，距离应接近 0）──')
    bad = 0
    for i, s in enumerate(samples[:8]):
        lid, dist = rec.predict(s)
        ok = (lid == label)
        bad += 0 if ok else 1
        print('    %d: id=%d dist=%.1f %s' % (i + 1, lid, dist, 'OK' if ok else '✗'))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
