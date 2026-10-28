# -*- coding: utf-8 -*-
"""fall_detect.py —— 老人跌倒检测（YOLOv5 模型 + JPEG 管道 IPC）。

结构照他们的做法：
  * 模型跑在 Python3 子进程（conda yolo 环境），通过 stdin/stdout 交换 JPEG；
  * 证据判定用他们的 FactoryFallEvidence（单人、score≥0.5、位移≤0.30、fall 连续 1 s）；
  * 主进程（py2/ROS）只负责抓帧、发帧、收结果，不碰底盘。

用法：
    det = FallDetector()
    det.start()                       # 起子进程并等 ready（模型加载 ~4 s）
    people = det.observe(bgr_image)   # 返回 [{'label','score','box','center'}]
    det.close()
"""
from __future__ import print_function

import json
import os
import select
import subprocess
import sys
import threading
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
WORKER = os.path.join(TOOLS, 'fall', 'factory_fall_worker.py')
WORKER_PY = '/home/ub/anaconda3/envs/yolo/bin/python'
LOGIC_DIR = os.path.join(TOOLS, 'fall')

if LOGIC_DIR not in sys.path:
    sys.path.insert(0, LOGIC_DIR)


def clean_env():
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    env.pop('PYTHONHOME', None)
    return env


class FallDetector(object):
    def __init__(self, worker=WORKER, worker_py=WORKER_PY):
        self.worker = worker
        self.worker_py = worker_py
        self.proc = None
        self.buf = b''
        self.seq = 0
        self.cv2 = None
        self.bridge = None

    # ---------------------------------------------------------- 进程
    def start(self, ready_timeout=30.0):
        if not os.path.isfile(self.worker):
            raise RuntimeError(u'找不到跌倒检测 worker：%s' % self.worker)
        if not os.path.isfile(self.worker_py):
            raise RuntimeError(u'找不到 conda 环境：%s' % self.worker_py)
        self.proc = subprocess.Popen(
            [self.worker_py, '-u', self.worker],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=0, env=clean_env())
        packet = self._read_packet(ready_timeout)
        if not packet.get('ready'):
            raise RuntimeError(u'worker 未就绪：%r' % packet)
        print(u'  [跌倒] 模型就绪：%s（conf=%s）'
              % (packet.get('backend'), packet.get('confidence')))
        return packet

    def _read_packet(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if b'\n' in self.buf:
                line, self.buf = self.buf.split(b'\n', 1)
                return json.loads(line.decode('utf-8'))
            if self.proc.poll() is not None:
                err = self.proc.stderr.read()[-400:] if self.proc.stderr else b''
                raise RuntimeError(u'worker 退出：%s' % err.decode('utf-8', 'replace'))
            if select.select([self.proc.stdout], [], [], 0.1)[0]:
                data = os.read(self.proc.stdout.fileno(), 65536)
                if not data:
                    raise RuntimeError(u'worker 输出结束')
                self.buf += data
        raise RuntimeError(u'worker 响应超时（%.0f s）' % timeout)

    # ---------------------------------------------------------- 单帧
    def observe(self, image_bgr, timeout=10.0):
        """送一帧 BGR 图，返回 people 列表。"""
        if self.proc is None or self.proc.poll() is not None:
            raise RuntimeError(u'worker 未运行')
        if self.cv2 is None:
            import cv2
            self.cv2 = cv2
        ok, encoded = self.cv2.imencode('.jpg', image_bgr,
                                        [self.cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError(u'JPEG 编码失败')
        self.seq += 1
        packet = {'seq': self.seq,
                  'jpeg': __import__('base64').b64encode(encoded.tostring()).decode('ascii')}
        self.proc.stdin.write((json.dumps(packet) + '\n').encode('ascii'))
        self.proc.stdin.flush()
        result = self._read_packet(timeout)
        if result.get('seq') != self.seq:
            raise RuntimeError(u'worker 返回序号不匹配')
        return result.get('people') or []

    def close(self):
        if self.proc is None:
            return
        try:
            if self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=3)
                except Exception:
                    self.proc.kill()
        except Exception:
            pass
        for stream in (self.proc.stdin, self.proc.stdout, self.proc.stderr):
            try:
                if stream:
                    stream.close()
            except Exception:
                pass
        self.proc = None


class LatestImage(object):
    """订阅相机，取最新一帧（BGR）。用法：img.get(timeout=2)。"""

    def __init__(self, topic='/camera/rgb/image_raw'):
        import rospy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.receipt = 0.0
        self.sub = rospy.Subscriber(topic, Image, self._cb, queue_size=1,
                                    buff_size=8 * 1024 * 1024)

    def _cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self.lock:
            self.frame = img
            self.receipt = time.time()

    def get(self, timeout=2.0):
        import rospy
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.frame is not None and time.time() - self.receipt < 2.0:
                    return self.frame
            time.sleep(0.05)
        return None

    def close(self):
        try:
            self.sub.unregister()
        except Exception:
            pass
