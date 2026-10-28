# -*- coding: utf-8 -*-
"""camera_warmup.py —— 等相机自动曝光稳定再发车。

为什么需要：Astra 节点刚起来时自动曝光要爬几秒到十几秒，这期间画面偏亮/偏暗，
二维码和人脸都会明显变差。m40 起栈后、发车之前调用本脚本，等画面均值稳定再走。

判据：每 0.5 s 采一次全图均值，最近 4 次采样的极差 < tol（默认 3/255）就算稳定；
另外至少等 min_wait 秒（默认 6 s），避免"一开始就恰好稳定"的假象。

用法：
    python2 camera_warmup.py                 # 默认 15 s 上限、tol 3.0、最少 6 s
    python2 camera_warmup.py --timeout 20 --tol 2.0 --min-wait 8
    python2 camera_warmup.py --quiet         # 只打印最后一行
退出码：0 = 已稳定；1 = 超时（仍继续，不阻塞流程）
"""
from __future__ import print_function

import argparse
import sys
import time

import rospy
from sensor_msgs.msg import Image

try:
    import numpy as np
except Exception:
    np = None


class Warmup(object):
    def __init__(self, topic):
        self.topic = topic
        self.mean = None
        self.count = 0
        self.t_last = 0.0
        rospy.Subscriber(topic, Image, self._cb, queue_size=1, buff_size=2 ** 24)

    def _cb(self, msg):
        try:
            if np is not None:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                self.mean = float(arr.mean())
            else:
                data = msg.data
                self.mean = float(sum(bytearray(data))) / max(1, len(data))
            self.count += 1
            self.t_last = time.time()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/rgb/image_raw')
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--tol', type=float, default=3.0)
    ap.add_argument('--min-wait', type=float, default=6.0, dest='min_wait')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    rospy.init_node('camera_warmup', anonymous=True)
    w = Warmup(args.topic)

    t0 = time.time()
    samples = []
    first = None
    while time.time() - t0 < args.timeout and not rospy.is_shutdown():
        time.sleep(0.5)
        if w.mean is None:
            continue
        if first is None:
            first = w.mean
        samples.append(w.mean)
        samples = samples[-4:]
        waited = time.time() - t0
        if len(samples) == 4 and waited >= args.min_wait:
            spread = max(samples) - min(samples)
            if not args.quiet:
                print(u'  采样 %d：亮度 %.1f（极差 %.1f，已等 %.1f s）'
                      % (w.count, w.mean, spread, waited))
            if spread < args.tol:
                print(u'  相机预热完成：%.1f s，亮度 %.1f -> %.1f'
                      % (waited, first, w.mean))
                return 0
    waited = time.time() - t0
    if w.mean is None:
        print(u'  [warn] 相机 %s 没有画面（等了 %.1f s），跳过预热' % (args.topic, waited))
        return 1
    print(u'  [warn] 预热超时（%.1f s，亮度 %.1f），仍继续' % (waited, w.mean))
    return 1


if __name__ == '__main__':
    sys.exit(main())
