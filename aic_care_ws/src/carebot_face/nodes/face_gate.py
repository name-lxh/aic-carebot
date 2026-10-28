#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
face_gate.py — 智慧养老护理赛题：人脸核验门禁节点

赛题要求
--------
「在离开客厅前，机器人需要在 A 点停留，进行人脸识别，若识别到预先指定的主人后，
  方可进入卧室、厨房和书房。」

这是一个**门槛条件**：不是识别到"有人"就行，必须识别到**指定的主人**。
参考实现 `face_recognition` 只发布「检测到的人脸 id 和置信度」，
**没有任何"是否为主人"的判定逻辑**，也没有与导航任务联动的接口 —— 本节点补上这一环。

输入（来自组委会提供的 face_recognition 节点）
---------------------------------------------
  /face_rst        std_msgs/Int16MultiArray
                   格式：[人脸数, id1, 置信度1, id2, 置信度2, ...]
                   · id == 255 表示未知人脸
                   · 置信度实为 100 - LBPH_distance，**越大越可信**
  /image_face      sensor_msgs/Image        带标注的画面

输出
----
  /face/verified    std_msgs/Bool     核验结果（latch 保持，供状态机查询）
  /face/status      std_msgs/String   人类可读状态（供语音播报）
  /face/debug_image sensor_msgs/Image 叠加了核验结论的画面（2 号机位演示用）

服务
----
  /face/verify         std_srvs/Trigger  执行一次核验（多帧投票），message 返回详情
  /face/reset          std_srvs/Trigger  清除核验结果（重新开始一轮）

设计要点
--------
1. **多帧投票，不靠单帧**：LBPH 对光照和角度很敏感。单帧判定会出现"闪一下就通过"
   或"明明对着却不过"。本节点在 T 秒窗口内采样 N 帧，要求主人 id 出现比例
   ≥ `~vote_ratio` 且平均置信度 ≥ `~min_confidence` 才判定通过。
2. **区分"没有人脸"与"不是主人"**：这两种失败要用不同的播报引导，
   「请正对摄像头」vs「未识别到主人」。赛题给 1 分钟的响应红线，播报引导很重要。
3. **声源定位联动（可选）**：若 voice 侧提供了唤醒方位角，可先转向说话人再核验，
   显著提高识别率。由 `~turn_to_speaker` 开关控制。
4. **降级可用**：若组委会的识别节点没起来，本节点可自己直接跑 LBPH 识别
   （`~self_recognition`），用同一份 trainer.yml 和 haarcascade。
"""

import sys
import os
import threading
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse

try:
    unicode  # noqa: F821  (Python 2)
except NameError:
    unicode = str  # noqa: A001


def setup_stdout():
    """让 stdout/stderr 能输出中文 —— Python 2.7 / 3.6 / 3.11 都能用。

    ⚠️ 这段代码修正了 4 次才对，动它之前请看完下面每一条：

    1) 不用 `hasattr(sys.stdout, 'reconfigure')` 判断 —— 那是 3.7+ 的方法。
       目标环境 Ubuntu 18.04 是 Python 3.6，hasattr 为假，设置被静默跳过，
       随后打印中文抛 UnicodeEncodeError。

    2) 不能只依赖 `stream.buffer` —— 那是 Python 3 的属性。
       Python 2.7（ROS Melodic 的实际运行版本）的 sys.stdout 没有它，
       getattr 得 None，同样静默跳过。

    3) 用 `codecs.getwriter` 包 Python 2 的 sys.stdout 时，**必须**处理
       字节串输入。因为 Python 2 里有两类代码往 stderr 写**字节串**：

           traceback.print_exc()     → file.write(str)  字节串
           logging.StreamHandler     → stream.write(msg + "\n")

       标准 StreamWriter 只接受 unicode，收到字节串会先 decode('ascii')，
       遇到非 ASCII 就崩：
           UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5
       后果是"为了看清异常而打印异常时崩掉"，真正的错误彻底看不到。
       —— 本项目实际踩过这个坑，排查了很久。

    4) 所以这里自定义 _Utf8Writer：字节串原样写，unicode 编码后写。

    实测环境：Python 3.11（本地开发）/ 3.6.9（Ubuntu 18.04 默认）
              / 2.7.17（ROS Melodic 跑节点的实际版本）
    """
    import codecs as _codecs

    class _Utf8Writer(_codecs.StreamWriter):
        """字节串与 unicode 都接受，都往底层流写 UTF-8 字节。"""

        def write(self, s):
            if s is None:
                return
            try:
                if isinstance(s, bytes):
                    # 已经是字节串（traceback / logging 会这样写）
                    self.stream.write(s)
                    return
                # unicode → UTF-8 字节
                self.stream.write(s.encode('utf-8', 'backslashreplace'))
            except Exception:
                # 兜底：尽全力输出，绝不因为编码问题让程序崩
                try:
                    self.stream.write(repr(s))
                except Exception:
                    pass

    def _wrap(stream):
        if stream is None:
            return stream
        # 已经是 UTF-8 的，不动
        enc = (getattr(stream, 'encoding', None) or '').lower().replace('-', '')
        if enc in ('utf8', 'utf_8'):
            return stream
        # 优先用底层字节流（Python 3）；Python 2 没有 .buffer，直接用流本身
        buf = getattr(stream, 'buffer', None)
        if buf is None:
            buf = stream
        try:
            return _Utf8Writer(buf, errors='backslashreplace')
        except Exception:
            pass
        # 退化：Python 3.7+ 的 reconfigure
        reconf = getattr(stream, 'reconfigure', None)
        if callable(reconf):
            try:
                reconf(encoding='utf-8', errors='backslashreplace')
            except Exception:
                pass
        return stream

    sys.stdout = _wrap(sys.stdout)
    sys.stderr = _wrap(sys.stderr)


def fix_ros_logging():
    """修复 ROS 日志打印中文崩溃的问题（Python 2 下必现）。

    背景
    ----
    ROS Melodic 跑在 Python 2.7 下，Python 2 的 `str(unicode)` 默认用
    **ASCII** 编码。而 ROS 的日志处理器就是这么转的：

        rosgraph/roslogging.py:249
            msg = msg.replace('${message}', str(record_message))

    于是
        rospy.loginfo(u'中文')
    直接抛
        UnicodeEncodeError: 'ascii' codec can't encode characters

    **任何节点用 rospy.log* 打中文都会崩**，而且常常崩在异常处理路径上，
    把真正的异常掩盖掉，调试时极具误导性。

    解法
    ----
    在 record 进入 handler **之前**，把 unicode 消息转成 UTF-8 字节串。
    这样 roslogging 里的 `str(record_message)` 变成"字节串转字节串"，
    是无操作，不会再触发 ASCII 编码。

    ⚠️ 这段代码必须保留 —— 去掉后所有中文日志都会崩。
    """
    import logging as _logging

    # 避免重复打补丁
    if getattr(_logging.Logger, '_ros_utf8_patched', False):
        return

    try:
        _text_types = (unicode,)                      # noqa: F821  (Py2)
    except NameError:
        _text_types = (str,)                          # Py3

    _orig_makeRecord = _logging.Logger.makeRecord

    def _utf8_makeRecord(self, name, level, fn, lno, msg, args, exc_info,
                         func=None, extra=None, sinfo=None):
        # 把 unicode 消息转成 UTF-8 字节串
        if isinstance(msg, _text_types):
            try:
                msg = msg.encode('utf-8')
            except Exception:
                pass
        # 参数里的 unicode 也要转，否则 % 格式化时同样会崩
        if args:
            try:
                args = tuple(
                    a.encode('utf-8') if isinstance(a, _text_types) else a
                    for a in args)
            except Exception:
                pass
        try:
            return _orig_makeRecord(self, name, level, fn, lno, msg, args,
                                    exc_info, func, extra, sinfo)
        except TypeError:
            # Python 2.7 的签名没有 sinfo
            return _orig_makeRecord(self, name, level, fn, lno, msg, args,
                                    exc_info, func, extra)

    _logging.Logger.makeRecord = _utf8_makeRecord
    _logging.Logger._ros_utf8_patched = True


def _u8(text):
    if isinstance(text, unicode):
        return text.encode('utf-8')
    return text


def _to_unicode(text):
    if isinstance(text, bytes):
        return text.decode('utf-8', 'replace')
    if isinstance(text, unicode):
        return text
    return unicode(text)


# 与参考实现 face_recognition.py 中的 names 表保持一致
DEFAULT_NAMES = ['None', 'id1', 'id2', 'id3', 'id4', 'id5', 'id6']


class FaceGate(object):

    def __init__(self):
        self.image_topic = rospy.get_param('~image_topic', '/image_face')
        self.rst_topic = rospy.get_param('~rst_topic', '/face_rst')

        # 主人 id 列表，逗号分隔，例如 "3" 或 "3,4"
        raw_ids = rospy.get_param('~master_ids', '3')
        self.master_ids = self._parse_ids(raw_ids)

        names = rospy.get_param('~names', DEFAULT_NAMES)
        self.names = names if isinstance(names, list) else DEFAULT_NAMES

        self.min_confidence = float(rospy.get_param('~min_confidence', 50.0))
        self.vote_ratio = float(rospy.get_param('~vote_ratio', 0.6))
        self.sample_window = float(rospy.get_param('~sample_window', 5.0))
        self.required_frames = int(rospy.get_param('~required_frames', 5))
        self.max_rounds = int(rospy.get_param('~max_rounds', 3))

        self.turn_to_speaker = bool(rospy.get_param('~turn_to_speaker', False))
        self.speaker_turn_speed = float(rospy.get_param('~speaker_turn_speed', 0.6))

        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')

        # 自识别模式（组委会识别节点缺席时的兜底）
        self.self_recognition = bool(rospy.get_param('~self_recognition', True))
        self.trainer_path = rospy.get_param(
            '~trainer_path',
            '/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/source/trainer/trainer.yml')
        self.cascade_path = rospy.get_param(
            '~cascade_path',
            '/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/'
            'haarcascade_frontalface_default.xml')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')

        # 状态
        self.lock = threading.RLock()
        self.verified = False
        self.latest_faces = []          # [(id, confidence), ...]
        self.last_rst_time = 0.0
        self.latest_annotated = None
        self.latest_raw_frame = None
        self._bridge = None
        self._cv2 = None
        self._recognizer = None
        self._cascade = None
        self._pub_vel = None

        self._init_vision()

        # 发布
        self.pub_verified = rospy.Publisher('/face/verified', Bool, queue_size=5, latch=True)
        self.pub_status = rospy.Publisher('/face/status', String, queue_size=10, latch=True)
        self.pub_debug = rospy.Publisher('/face/debug_image', Image, queue_size=2)

        # 订阅
        self.sub_rst = rospy.Subscriber(self.rst_topic, Int16MultiArray, self.on_face_rst, queue_size=20)
        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.on_annotated, queue_size=2,
                                        buff_size=2 ** 24)
        if self.self_recognition:
            self.sub_cam = rospy.Subscriber(self.camera_topic, Image, self.on_camera, queue_size=2,
                                            buff_size=2 ** 24)

        # 服务
        self.srv_verify = rospy.Service('/face/verify', Trigger, self.on_verify)
        self.srv_reset = rospy.Service('/face/reset', Trigger, self.on_reset)

        self._set_status(u'人脸核验门禁就绪（主人 id = %s，置信度阈值 = %.0f）'
                         % (self.master_ids, self.min_confidence))
        self._check_inputs()

    # ------------------------------------------------------------ 初始化
    @staticmethod
    def _parse_ids(raw):
        out = []
        for token in _to_unicode(raw).replace(u'，', u',').split(','):
            token = token.strip()
            if not token:
                continue
            try:
                out.append(int(token))
            except ValueError:
                rospy.logwarn('[face_gate] 无法解析主人 id：%s', token)
        return out or [3]

    def _init_vision(self):
        try:
            import cv2
            import cv_bridge
            self._cv2 = cv2
            self._bridge = cv_bridge.CvBridge()
        except ImportError as exc:
            rospy.logwarn('[face_gate] OpenCV / cv_bridge 不可用（%s），'
                          '将只依赖组委会识别节点的结果', exc)
            self.self_recognition = False
            return

        if not self.self_recognition:
            return

        try:
            if not os.path.isfile(self.trainer_path):
                rospy.logwarn('[face_gate] 找不到训练器文件：%s，自识别已禁用', self.trainer_path)
                self.self_recognition = False
                return
            # ★ OpenCV 3.x 的工厂是 createLBPHFaceRecognizer()，
            #   只有 OpenCV 4.x 才有 LBPHFaceRecognizer_create()；
            #   模型读取 3.x 叫 load()，4.x 叫 read()。这台车是 3.2.0。
            _face = self._cv2.face
            if hasattr(_face, 'LBPHFaceRecognizer_create'):
                self._recognizer = _face.LBPHFaceRecognizer_create()
            else:
                self._recognizer = _face.createLBPHFaceRecognizer()
            if hasattr(self._recognizer, 'read'):
                self._recognizer.read(self.trainer_path)
            else:
                self._recognizer.load(self.trainer_path)
            if not os.path.isfile(self.cascade_path):
                rospy.logwarn('[face_gate] 找不到 Haar 级联文件：%s，自识别已禁用', self.cascade_path)
                self.self_recognition = False
                return
            self._cascade = self._cv2.CascadeClassifier(self.cascade_path)
            rospy.loginfo('[face_gate] 自识别已启用（trainer=%s）', self.trainer_path)
        except Exception as exc:
            rospy.logwarn('[face_gate] 自识别初始化失败：%s', exc)
            self.self_recognition = False

    def _check_inputs(self):
        rospy.Timer(rospy.Duration(6.0), self._probe_topics, oneshot=True)

    def _probe_topics(self, _evt):
        try:
            published = [t[0] for t in rospy.get_published_topics()]
        except Exception:
            return
        if self.rst_topic not in published and not self.self_recognition:
            self._set_status(u'警告：识别节点未发布 %s，且自识别不可用 —— '
                             u'请启动 face_recognition 或检查 trainer.yml 路径' % self.rst_topic)
        elif self.rst_topic in published:
            self._set_status(u'检测到组委会识别节点在发布 %s' % self.rst_topic)

    # ------------------------------------------------------------ 订阅回调
    def on_face_rst(self, msg):
        """解析 [人脸数, id1, 置信度1, ...]"""
        data = list(msg.data)
        if not data:
            return
        count = int(data[0])
        faces = []
        for i in range(count):
            idx = 1 + i * 2
            if idx + 1 >= len(data):
                break
            fid = int(data[idx])
            conf = float(data[idx + 1])
            if fid == 255:
                faces.append((255, conf))
            else:
                faces.append((fid, conf))
        with self.lock:
            self.latest_faces = faces
            self.last_rst_time = time.time()

    def on_annotated(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self.lock:
            self.latest_annotated = frame
        self._publish_debug(frame)

    def on_camera(self, msg):
        """自识别路径：直接对原始相机图像跑 LBPH。"""
        if not self.self_recognition or self._recognizer is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self.lock:
            self.latest_raw_frame = frame

        h, w = frame.shape[:2]
        if h < 240 or w < 320:
            return
        try:
            gray = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2GRAY)
            min_w = int(0.1 * w)
            min_h = int(0.1 * h)
            faces = self._cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5,
                                                   minSize=(min_w, min_h))
            parsed = []
            display = frame.copy()
            for (x, y, fw, fh) in faces:
                fid, dist = self._recognizer.predict(gray[y:y + fh, x:x + fw])
                conf = 100.0 - float(dist)     # 与参考实现口径一致：越大越可信
                if dist >= 100:
                    fid = 255
                parsed.append((int(fid), conf))
                color = (0, 255, 0) if (fid in self.master_ids and dist < 100) else (0, 165, 255)
                self._cv2.rectangle(display, (x, y), (x + fw, y + fh), color, 2)
                label = u'%s %.0f%%' % (self._name_of(fid), conf)
                self._cv2.putText(display, _u8(label), (x + 5, y - 5),
                                  self._cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            with self.lock:
                if parsed:
                    self.latest_faces = parsed
                    self.last_rst_time = time.time()
                self.latest_annotated = display
            self._publish_debug(display)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, '[face_gate] 自识别出错：%s', exc)

    def _publish_debug(self, frame):
        try:
            self.pub_debug.publish(self._bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception:
            pass

    # ------------------------------------------------------------ 判定
    def _name_of(self, fid):
        if fid == 255:
            return u'未知'
        if 0 <= fid < len(self.names):
            return _to_unicode(self.names[fid])
        return u'id%d' % fid

    def _evaluate(self, frames):
        """对采样窗口内的多帧结果投票，返回 (passed, detail_text)。

        frames: [[(id, conf), ...], ...]
        """
        total_faces = 0
        unknown_faces = 0
        master_hits = 0
        master_conf_sum = 0.0
        per_round_best = []

        for faces in frames:
            if not faces:
                per_round_best.append(None)
                continue
            total_faces += len(faces)
            best_master = None
            for fid, conf in faces:
                if fid == 255:
                    unknown_faces += 1
                    continue
                if fid in self.master_ids and conf >= self.min_confidence:
                    master_hits += 1
                    master_conf_sum += conf
                    if best_master is None or conf > best_master[1]:
                        best_master = (fid, conf)
            per_round_best.append(best_master)

        rounds_with_face = sum(1 for f in frames if f)
        rounds_with_master = sum(1 for b in per_round_best if b is not None)

        if rounds_with_face == 0:
            return False, u'未检测到任何人脸，请正对摄像头'

        ratio = float(rounds_with_master) / float(rounds_with_face)
        avg_conf = (master_conf_sum / master_hits) if master_hits else 0.0

        detail = (u'采样 %d 轮，有脸 %d 轮，命中主人 %d 轮（比例 %.0f%%），'
                  u'平均置信度 %.0f，检出人脸 %d 张（其中未知 %d 张）'
                  % (len(frames), rounds_with_face, rounds_with_master,
                     ratio * 100.0, avg_conf, total_faces, unknown_faces))

        if ratio >= self.vote_ratio and avg_conf >= self.min_confidence:
            return True, detail
        if total_faces > 0 and unknown_faces == total_faces:
            return False, u'检测到人脸但不是主人。' + detail
        return False, detail

    def _turn_to_speaker(self):
        """（可选）先转向声源方向，提高识别率。

        依赖 voice_bridge 转发的 /voice/awake_angle（麦克风阵列的 DOA 结果，
        1° 分辨率）。拿不到角度就静默跳过，不影响核验主流程。
        """
        if not self.turn_to_speaker:
            return
        try:
            from std_msgs.msg import Int32
            angle_msg = rospy.wait_for_message('/voice/awake_angle', Int32, timeout=1.0)
        except Exception:
            return

        angle = float(angle_msg.data)
        # 归一化到 [-180, 180]
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0

        if abs(angle) < 10.0:
            return

        try:
            from geometry_msgs.msg import Twist
            if self._pub_vel is None:
                self._pub_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=5)
            import math
            yaw = math.radians(angle)
            speed = self.speaker_turn_speed if yaw > 0 else -self.speaker_turn_speed
            twist = Twist()
            twist.angular.z = speed
            duration = abs(yaw) / abs(speed)
            rate = rospy.Rate(20)
            end = time.time() + min(duration, 3.0)
            while time.time() < end and not rospy.is_shutdown():
                self._pub_vel.publish(twist)
                rate.sleep()
            self._pub_vel.publish(Twist())     # 停车
            rospy.sleep(0.3)
            self._set_status(u'已转向声源方向 %.0f°' % angle)
        except Exception as exc:
            rospy.logwarn('[face_gate] 转向声源失败：%s', exc)

    # ------------------------------------------------------------ 服务
    def on_verify(self, _req):
        self._turn_to_speaker()

        frames = []
        rate = rospy.Rate(5)
        for _r in range(self.max_rounds):
            frames = []
            deadline = time.time() + self.sample_window
            while time.time() < deadline and not rospy.is_shutdown():
                with self.lock:
                    faces = list(self.latest_faces)
                    age = time.time() - self.last_rst_time
                # 只采纳新鲜的结果（1 秒内）
                frames.append(faces if age < 1.0 else [])
                rate.sleep()

            passed, detail = self._evaluate(frames)
            if passed:
                self.verified = True
                self.pub_verified.publish(Bool(data=True))
                text = u'已识别到主人（%s）' % self._name_of(self.master_ids[0])
                self._set_status(text + u'。' + detail)
                return TriggerResponse(success=True, message=_u8(text + u'|' + detail))

            self._set_status(u'第 %d 轮核验未通过：%s' % (_r + 1, detail))

        # 三轮都失败：给出针对性的引导播报
        passed, detail = self._evaluate(frames)
        self.verified = False
        self.pub_verified.publish(Bool(data=False))
        return TriggerResponse(success=False, message=_u8(u'人脸核验未通过。' + detail))

    def on_reset(self, _req):
        with self.lock:
            self.verified = False
            self.latest_faces = []
            self.last_rst_time = 0.0
        self.pub_verified.publish(Bool(data=False))
        self._set_status(u'核验结果已清除')
        return TriggerResponse(success=True, message=_u8(u'已清除核验结果'))

    # ------------------------------------------------------------ 工具
    def _set_status(self, text):
        self.pub_status.publish(String(data=_u8(text)))
        rospy.loginfo('[face_gate] %s', _u8(text))


if __name__ == '__main__':
    setup_stdout()
    fix_ros_logging()
    rospy.init_node('face_gate')
    try:
        FaceGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
