# -*- coding: utf-8 -*-
"""retrain_face.py —— 用【照片 + 现场采集帧】重建人脸模型（只留主人一个 id）

    python2 retrain_face.py <照片目录> <现场帧目录> [label] [目标尺寸] [现场用几张]

为什么这样设计：
  * 照片：多角度、有姿态多样性，但和相机视角有差异
  * 现场帧：相机原始裁剪，尺度/光照/视角完全对得上
  两者混在一起训练，既覆盖姿态又贴合相机。

为什么"重建"而不是"追加"：
  追加会让样本无限膨胀（每个 116 KB，几百个就是几十 MB），而且旧的错误样本
  （比如之前用 654 px 证件照训的）会一直占着位置。重建能用最少的好样本达到
  最低的距离。

输出：
  * 新模型写到 ~/Hajimi/LXH/face_dataset/model/trainer.yml（我们自己的目录）
  * 打印留一验证的距离分布（用其它现场帧训练、预测留下的那一帧），
    这才是"现场真人"的诚实估计
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
OUT = '/home/ub/Hajimi/LXH/face_dataset/model/trainer.yml'


def faces_from(paths, casc, target):
    out = []
    for f in paths:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        h, w = img.shape[:2]
        det = cv2.equalizeHist(img)
        boxes = casc.detectMultiScale(det, 1.15, 4,
                                      minSize=(int(0.08 * w), int(0.08 * h)))
        if len(boxes) == 0:
            boxes = casc.detectMultiScale(det, 1.1, 3)
        if len(boxes) == 0:
            # 现场帧本身可能就是人脸裁剪，整张当脸用
            if 0.6 <= float(w) / h <= 1.6 and w >= 60:
                out.append((f, cv2.resize(img, (target, target),
                                          interpolation=cv2.INTER_AREA)))
            continue
        x, y, fw, fh = max(boxes, key=lambda r: r[2] * r[3])
        out.append((f, cv2.resize(img[y:y + fh, x:x + fw], (target, target),
                                  interpolation=cv2.INTER_AREA)))
    return out


def variants(face):
    out = [face, cv2.flip(face, 1)]
    t = face.shape[0]
    for ang in (-8, 8):
        m = cv2.getRotationMatrix2D((t / 2.0, t / 2.0), ang, 1.0)
        out.append(cv2.warpAffine(face, m, (t, t), flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_REPLICATE))
    for d in (12, -12):
        out.append(np.clip(face.astype(np.int16) + d, 0, 255).astype(np.uint8))
    return out


def main():
    photos_dir = sys.argv[1]
    live_dir = sys.argv[2]
    label = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    target = int(sys.argv[4]) if len(sys.argv) > 4 else 115
    n_live = int(sys.argv[5]) if len(sys.argv) > 5 else 40

    casc = cv2.CascadeClassifier(CASCADE)

    def expand(d):
        fs = []
        for ext in ('*.jpg', '*.jpeg', '*.png', '*.JPG'):
            fs.extend(glob.glob(os.path.join(d, ext)))
        return sorted(set(fs))

    photo_files = expand(photos_dir)
    live_files = expand(live_dir)
    print('  照片 %d 张，现场帧 %d 张' % (len(photo_files), len(live_files)))

    photos = faces_from(photo_files, casc, target)
    print('  照片里检出人脸 %d 张' % len(photos))

    # 现场帧均匀抽样 n_live 张
    if len(live_files) > n_live:
        step = len(live_files) / float(n_live)
        live_files = [live_files[int(i * step)] for i in range(n_live)]
    lives = faces_from(live_files, casc, target)
    print('  现场帧里可用 %d 张（抽 %d）' % (len(lives), len(live_files)))

    if len(lives) < 5:
        print('  [FAIL] 现场帧太少，无法留一验证')
        return 1

    # ---- 留一验证：照片 + 其它现场帧 -> 预测留下的那帧 ----
    base_train = [f for _, f in photos]
    dists = []
    for i in range(len(lives)):
        tr = base_train + [f for j, (_, f) in enumerate(lives) if j != i]
        rec = cv2.face.createLBPHFaceRecognizer()
        rec.train(tr, np.array([label] * len(tr), dtype=np.int32))
        _, d = rec.predict(lives[i][1])
        dists.append(d)
    sd = sorted(dists)
    print('')
    print('  ── 留一验证（现场帧，最接近真人的指标）──')
    print('    距离: %s' % ' '.join('%.1f' % d for d in sd))
    print('    最小 %.1f  中位 %.1f  最大 %.1f   -> 置信度 中位 %.1f'
          % (sd[0], sd[len(sd) // 2], sd[-1], 100.0 - sd[len(sd) // 2]))

    # ---- 正式训练 ----
    samples, labels = [], []
    for _, f in photos + lives:
        for v in variants(f):
            samples.append(v)
            labels.append(label)
    print('')
    print('  训练样本 %d 个（label=%d）' % (len(samples), label))

    rec = cv2.face.createLBPHFaceRecognizer()
    rec.train(samples, np.array(labels, dtype=np.int32))
    if not os.path.isdir(os.path.dirname(OUT)):
        os.makedirs(os.path.dirname(OUT))
    if os.path.isfile(OUT):
        shutil.copy2(OUT, OUT + '.bak_' + time.strftime('%H%M%S'))
    rec.save(OUT)
    print('  已保存: %s（%.1f MB）'
          % (OUT, os.path.getsize(OUT) / 1e6))
    return 0


if __name__ == '__main__':
    sys.exit(main())
