#!/usr/bin/env python3
"""Factory YOLOv5 fall model with private JPEG IPC; no ROS/motion/audio/files."""
import base64
import contextlib
import hashlib
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FACTORY = '/home/ub/yolov5-mask-42-master'
MODEL = os.path.join(HERE, 'factory_fall_assets', 'best.pt')
MODEL_SHA256 = 'df1eab425a411d67e2596e4744ef9ebda3d41d3b2a7216fcba4f2b7aad5f333f'
CONFIDENCE = .50


class FactoryFallModel:
    def __init__(self):
        if hashlib.sha256(open(MODEL, 'rb').read()).hexdigest() != MODEL_SHA256:
            raise RuntimeError('Factory fall model checksum mismatch')
        sys.path.insert(0, FACTORY)
        os.environ['YOLOV5_CONFIG_DIR'] = os.path.join(HERE, 'factory_fall_assets')
        # Factory modules may print startup text; stdout is reserved for IPC.
        with contextlib.redirect_stdout(sys.stderr):
            import cv2
            import numpy as np
            import torch
            from detect1 import DetectMultiBackend, non_max_suppression, scale_coords, check_img_size
            from utils.augmentations import letterbox
            torch.set_num_threads(2)
            cv2.setNumThreads(1)
            self.cv2, self.np, self.torch = cv2, np, torch
            self.nms, self.scale, self.letterbox = non_max_suppression, scale_coords, letterbox
            self.model = DetectMultiBackend(MODEL, device=torch.device('cpu'), dnn=False)
            self.model.model.float().eval()
            self.stride = self.model.stride
            self.size = check_img_size([640, 640], s=self.stride)
            self.names = self.model.names
            labels = list(self.names.values()) if isinstance(self.names, dict) else list(self.names)
            if set(str(x).strip().lower() for x in labels) != set(['fall', 'no fall']):
                raise RuntimeError('Unexpected factory fall model labels: '+repr(labels))

    def infer(self, image):
        cv2, np, torch = self.cv2, self.np, self.torch
        h, w = image.shape[:2]
        # Same letterbox, BGR->RGB, normalization and NMS as factory detect1.py.
        letterboxed = self.letterbox(image, self.size, stride=self.stride, auto=True)[0]
        im = np.ascontiguousarray(letterboxed.transpose((2, 0, 1))[::-1])
        tensor = torch.from_numpy(im).float().div_(255.)[None]
        with torch.no_grad(), contextlib.redirect_stdout(sys.stderr):
            pred = self.model(tensor, augment=False, visualize=False)
            det = self.nms(pred, CONFIDENCE, .45, None, False, max_det=1000)[0]
        people = []
        if len(det):
            det[:, :4] = self.scale(tensor.shape[2:], det[:, :4], image.shape).round()
            for row in det.cpu().tolist():
                x1, y1, x2, y2, score, cls = row
                label = str(self.names[int(cls)]).strip().lower()
                people.append({'label': label, 'score': float(score), 'box': [x1,y1,x2,y2],
                               'center': [(x1+x2)/(2*w), (y1+y2)/(2*h)]})
        return people


def emit(packet):
    sys.stdout.write(json.dumps(packet, ensure_ascii=True)+'\n')
    sys.stdout.flush()


def main():
    model = FactoryFallModel()
    if '--self-test' in sys.argv:
        t = time.time()
        people = model.infer(model.np.zeros((480,640,3), dtype=model.np.uint8))
        emit({'self_test': True, 'people': people, 'backend': 'factory_yolov5',
              'confidence': CONFIDENCE, 'seconds': time.time()-t, 'labels': model.names})
        return
    emit({'ready': True, 'backend': 'factory_yolov5', 'confidence': CONFIDENCE})
    for line in sys.stdin:
        packet = json.loads(line)
        raw = base64.b64decode(packet['jpeg'], validate=True)
        if len(raw) > 5000000:
            raise RuntimeError('Camera packet too large')
        image = model.cv2.imdecode(model.np.frombuffer(raw, dtype=model.np.uint8), model.cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('Invalid camera JPEG')
        started = time.time()
        people = model.infer(image)
        emit({'seq': packet['seq'], 'people': people, 'backend': 'factory_yolov5',
              'inference_seconds': time.time()-started})


if __name__ == '__main__':
    main()
