#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
qr_detector.py — 智慧养老护理赛题：二维码巡视顺序识别节点

赛题要求
--------
机器人在二维码张贴区域停留，读取二维码中编码的"后面三个房间的巡视顺序"，
然后按该顺序巡视 B / C / D 三个点。二维码约 200mm×200mm，张贴位置现场随机公布。

设计要点
--------
1. **多尺度检测**：200mm 的码在 2m 外只占很少像素，需上采样后二次检测。
2. **CLAHE 增强**：场地光照可能偏暗或被裁判改变，自适应直方图均衡化显著提升成功率。
3. **帧级去抖**：同一 payload 连续 N 帧一致才接受，避免单帧误读导致顺序全错。
4. **宽容解析**：现场公布的编码格式未知，解析器同时支持数字串/字母串/中文/JSON/URL。
5. **深度辅助**：融合深度信息估算距离，供导航层调整目标点。
6. **人工兜底**：提供 /qr/force_order 服务，可在调试或极端故障时手动注入顺序。

发布
----
  /qr/detection        std_msgs/String            二维码原始载荷
  /qr/room_order       std_msgs/Int16MultiArray   解析出的房间顺序 (2=B 3=C 4=D)
  /qr/debug_image      sensor_msgs/Image          带检测框的调试图（2 号机位用）
  /qr/status           std_msgs/String            人类可读状态（供语音播报）

服务
----
  /qr/detect_once      std_srvs/Trigger           阻塞式执行一次识别，success=是否成功

参数
----
  ~image_topic         默认 /camera/rgb/image_raw
  ~depth_topic         默认 /camera/depth/image_raw
  ~roi_top_ratio       默认 0.35   只处理图像下半部（二维码贴低处）
  ~confirm_frames      默认 3      连续一致帧数阈值
  ~max_attempts        默认 30     超时前的最大尝试帧数
  ~upscale             默认 2.0    低分辨率二次检测的上采样倍数
  ~enable_clahe        默认 True
  ~use_depth           默认 True
  ~light_trigger_topic 默认 ''     识别前可触发补光的 topic（可选）
"""

import sys

# "\u5b57\u8282\u4e32\u63d2\u8fdb unicode \u683c\u5f0f\u4e32"\u5728 Py2 \u4e0b\u4f1a\u6309 ASCII \u89e3\u7801\u800c\u5d29\uff0c
# \u6240\u4ee5\u53c2\u6570\u8981\u5148\u8f6c\u6210 unicode\u3002
# \uff08\u8be6\u89c1 utf8_stdio.to_text \u7684\u8bf4\u660e\uff09
try:
    from utf8_stdio import to_text
except ImportError:
    def to_text(v):
        if v is None:
            return u''
        if isinstance(v, bytes):
            try:
                return v.decode('utf-8')
            except Exception:
                return v.decode('utf-8', 'replace')
        return u'%s' % (v,)

import json
import re
import threading
import time

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse

# Python 2.7 / 3.x 兼容（机器人运行时为 Ubuntu 16.04 + ROS Kinetic + Python 2.7）
try:
    unicode  # noqa: F821  (Python 2)
except NameError:  # Python 3
    unicode = str  # noqa: A001
    bytes = bytes

try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False

try:
    import zbar
    HAS_ZBAR = True
except ImportError:
    HAS_ZBAR = False


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


# ---------------------------------------------------------------------------
# 房间编号约定：与赛题图二的 B / C / D 点一一对应
# ---------------------------------------------------------------------------
ROOM_B = 2   # 卧室
ROOM_C = 3   # 厨房
ROOM_D = 4   # 书房

ROOM_NAME = {
    ROOM_B: u'卧室',
    ROOM_C: u'厨房',
    ROOM_D: u'书房',
}

# 中文房间名 -> 编号
CN_NAME_TO_ID = {
    u'卧室': ROOM_B, u'睡房': ROOM_B, u'房间b': ROOM_B,
    u'厨房': ROOM_C, u'餐厅': ROOM_C, u'房间c': ROOM_C,
    u'书房': ROOM_D, u'客厅': 1, u'房间d': ROOM_D,
}


def parse_room_order(payload):
    """宽容解析二维码载荷，返回 (order_list, reason)。

    order_list 为 [2,3,4] 这种 B/C/D 编号序列；失败时返回 (None, 失败原因)。

    支持的格式（按优先级）：
        1. JSON        {"order": [2,3,4]} / {"rooms":[2,3,4]} / [2,3,4]
        2. URL 查询串  http://x/y?order=234&...
        3. 分隔数字    "2,3,4" / "2-3-4" / "2 3 4"
        4. 紧凑数字    "234"
        5. 字母序列    "B,C,D" / "BCD" / "b-c-d"
        6. 中文房间名  "卧室,厨房,书房"
    """
    if payload is None:
        return None, u'载荷为空'

    raw = payload.strip()
    if not raw:
        return None, u'载荷为空'

    # ---- 1. JSON -----------------------------------------------------------
    if raw.startswith('{') or raw.startswith('['):
        try:
            obj = json.loads(raw)
            if isinstance(obj, list):
                vals = obj
            elif isinstance(obj, dict):
                vals = None
                for key in ('order', 'rooms', 'room_order', 'sequence', 'seq', 'path'):
                    if key in obj:
                        vals = obj[key]
                        break
                if vals is None:
                    return None, u'JSON 中未找到 order/rooms 字段'
            else:
                return None, u'JSON 根类型不支持: %s' % type(obj).__name__
            order = _normalize_sequence(vals)
            if order:
                return order, 'json'
            # ⚠️ %r 产出字节串，插进 unicode 格式串时 Py2 按 ASCII
            #    解码它 —— vals 里含非 ASCII 就崩。
            return None, (u'JSON 序列无法归一化: %s'
                           % to_text(vals))
        except ValueError as exc:
            return None, u'JSON 解析失败: %s' % repr(exc)

    # ---- 2. URL 查询串 -----------------------------------------------------
    if '://' in raw or '?' in raw:
        m = re.search(r'[?&](order|rooms|seq)=([^&#]+)', raw)
        if m:
            order = _normalize_sequence(m.group(2))
            if order:
                return order, 'url-query'
        # 退化：从整条 URL 里抓字母
        order = _extract_letters(raw)
        if order:
            return order, 'url-letters'
        return None, u'URL 中未找到顺序信息'

    # ---- 3/4. 数字序列 -----------------------------------------------------
    digits = re.findall(r'\d', raw)
    if digits:
        order = _normalize_sequence(digits)
        if order:
            return order, 'digits'
        # ⚠️ 同上：digits 来自 OCR/解析，可能有非 ASCII
        return None, (u'数字序列 %s 不是 B/C/D 的排列'
                       % to_text(''.join(digits)))

    # ---- 5. 字母序列 -------------------------------------------------------
    order = _extract_letters(raw)
    if order:
        return order, 'letters'

    # ---- 6. 中文房间名 -----------------------------------------------------
    tokens = []
    for token in re.split(u'[,，、;；\\s]+', raw):
        token = token.strip().lower()
        if token and token in CN_NAME_TO_ID:
            tokens.append(token)
    order = _strict_normalize(tokens)
    if order:
        return order, 'chinese'

    # ⚠️ `raw` 是二维码的**原始载荷**，可能含任意字节（不是 UTF-8
    #    也正常）。用 %r 的话它产出字节串，插进 unicode 格式串时
    #    Py2 会按 ASCII 解码 → 崩。而且这行正在"识别失败"的分支里，
    #    一崩就看不到失败原因了。
    return None, (u'无法识别的载荷: %s'
                   % to_text(raw))


def _extract_letters(text):
    """从文本里抽取 B/C/D 字母（大小写均可），做严格校验。"""
    letters = re.findall(r'[BbCcDd]', text)
    if not letters:
        return None
    return _strict_normalize(letters)


def _to_text(v):
    """把任意标量安全转成 unicode 文本（兼容 Python 2.7 / 3.x）。"""
    if isinstance(v, bytes):
        return v.decode('utf-8', 'replace')
    if isinstance(v, unicode):
        return v
    return unicode(v)


def _u8(text):
    """统一转成 UTF-8 字节串。

    Python 2.7 下 ROS 的 std_msgs/String 只接受 ASCII 兼容的 str，
    直接塞 unicode 会在序列化时抛 UnicodeEncodeError，所以必须显式编码。
    """
    if isinstance(text, unicode):
        return text.encode('utf-8')
    return text


def _normalize_sequence(vals):
    """把各种取值统一成 [2,3,4] 形式的列表（严格校验，见 _strict_normalize）。"""
    if isinstance(vals, (str, bytes, unicode)):
        vals = list(vals)
    return _strict_normalize(vals)


def _strict_normalize(vals):
    """严格归一化：必须恰好是 B/C/D 的一个排列，否则返回 None。

    赛题要求巡视三个房间，所以合法顺序必须正好包含 2、3、4 各一次。
    宁可判定"解析失败"（会播报原文，允许人工兜底），也不能接受一个错误的顺序 ——
    顺序错了会导致后续 30 分的房间任务全部走错，损失远大于一次识别失败。
    """
    out = []
    for v in vals:
        if isinstance(v, bool):
            continue
        if isinstance(v, int):
            out.append(v)
            continue
        s = _to_text(v).strip()
        if not s:
            continue
        if s.isdigit():
            out.append(int(s))
        elif s.upper() in ('B', 'C', 'D'):
            out.append({'B': ROOM_B, 'C': ROOM_C, 'D': ROOM_D}[s.upper()])
        elif s in CN_NAME_TO_ID:
            out.append(CN_NAME_TO_ID[s])
        else:
            return None

    # 长度必须恰为 3，且恰好是 {2, 3, 4}（这一步同时排除了重复值）
    if len(out) != 3:
        return None
    if set(out) != set((ROOM_B, ROOM_C, ROOM_D)):
        return None
    return out


def _dedup_bcd(seq):
    """保序去重，只保留 2/3/4。"""
    seen = set()
    out = []
    for x in seq:
        if x in (ROOM_B, ROOM_C, ROOM_D) and x not in seen:
            seen.add(x)
            out.append(x)
    return out


def order_to_text(order):
    """把顺序列表转成中文播报文本。"""
    if not order:
        return u'未知顺序'
    return u'、'.join(ROOM_NAME.get(x, str(x)) for x in order)


# ---------------------------------------------------------------------------
# 主节点
# ---------------------------------------------------------------------------
class QrDetector(object):

    def __init__(self):
        if not (HAS_PYZBAR or HAS_ZBAR):
            rospy.logfatal('未找到 pyzbar / zbar 库。请先安装：')
            rospy.logfatal('  sudo apt-get install -y libzbar0 python-zbar')
            rospy.logfatal('  pip install pyzbar')
            raise RuntimeError('missing zbar backend')

        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.roi_top_ratio = float(rospy.get_param('~roi_top_ratio', 0.35))
        self.confirm_frames = int(rospy.get_param('~confirm_frames', 3))
        self.max_attempts = int(rospy.get_param('~max_attempts', 30))
        self.upscale = float(rospy.get_param('~upscale', 2.0))
        self.enable_clahe = bool(rospy.get_param('~enable_clahe', True))
        self.use_depth = bool(rospy.get_param('~use_depth', True))
        self.publish_debug = bool(rospy.get_param('~publish_debug', True))

        self.bridge = cv_bridge.CvBridge()
        self.lock = threading.RLock()

        # 去抖状态
        self._last_payload = None
        self._payload_count = 0
        self._decode_attempts = 0
        self._depth_frame = None
        self._last_distance = None

        # 结果缓存
        self._order = None
        self._raw_payload = None
        self._done = False

        # 发布者
        self.pub_detection = rospy.Publisher('/qr/detection', String, queue_size=5, latch=True)
        self.pub_order = rospy.Publisher('/qr/room_order', Int16MultiArray, queue_size=5, latch=True)
        self.pub_status = rospy.Publisher('/qr/status', String, queue_size=5, latch=True)
        if self.publish_debug:
            self.pub_debug = rospy.Publisher('/qr/debug_image', Image, queue_size=2)
        else:
            self.pub_debug = None

        # 可选的补光触发（发布空 String，由外部节点/GPIO 处理）
        light_topic = rospy.get_param('~light_trigger_topic', '')
        self.pub_light = rospy.Publisher(light_topic, String, queue_size=1) if light_topic else None

        self.sub_image = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=2,
                                          buff_size=2 ** 24)
        if self.use_depth:
            self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.on_depth, queue_size=2,
                                              buff_size=2 ** 24)

        self.srv_detect = rospy.Service('/qr/detect_once', Trigger, self.on_detect_once)
        self.srv_force = rospy.Service('/qr/force_order', Trigger, self.on_force_order)

        self._set_status(u'二维码识别节点就绪，等待图像')
        rospy.loginfo('[qr_detector] 就绪。后端=%s 图像话题=%s',
                      'pyzbar' if HAS_PYZBAR else 'zbar', self.image_topic)

    # -- 内部工具 -----------------------------------------------------------
    def _set_status(self, text):
        self.pub_status.publish(String(data=_u8(text)))
        rospy.loginfo('[qr_detector] %s', _u8(text))

    def _publish_result(self, order, payload, source):
        msg = Int16MultiArray()
        msg.data = [int(x) for x in order]
        self.pub_order.publish(msg)
        self.pub_detection.publish(String(data=payload))
        self._set_status(u'二维码识别成功（%s）：顺序 = %s' % (source, order_to_text(order)))
        rospy.loginfo('[qr_detector] room_order = %s', msg.data)

    # -- 解码 ---------------------------------------------------------------
    def _pick(self, results, shape):
        """从一帧里的多张码中挑"用户举在镜头前的那一张"。

        2026-10-20 现场：场地里同时有多张二维码，原实现直接 `return results[0]`，
        等于"pyzbar 先吐哪个就认哪个" —— 同一轮里先后读到了 BCD / CBD / DCB，
        比赛时会报错顺序。改成按"**离画面中心近 + 面积大**"打分。

        打分 = 面积归一化(0~1) * 0.6  +  居中程度(0~1) * 0.4
        """
        h, w = shape[:2]
        cx0, cy0 = w / 2.0, h / 2.0
        best, best_score = None, -1.0
        for payload, pts in results:
            if pts is None:
                area_ratio, center_ratio = 0.0, 0.0
            else:
                xs = [float(p[0]) for p in pts]
                ys = [float(p[1]) for p in pts]
                bw = max(xs) - min(xs)
                bh = max(ys) - min(ys)
                area_ratio = min(1.0, (bw * bh) / float(w * h) * 6.0)
                ccx, ccy = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
                dist = ((ccx - cx0) ** 2 + (ccy - cy0) ** 2) ** 0.5
                center_ratio = max(0.0, 1.0 - dist / ((w * w + h * h) ** 0.5 / 2.0))
            score = 0.6 * area_ratio + 0.4 * center_ratio
            if score > best_score:
                best, best_score = (payload, pts), score
        if best is not None and len(results) > 1:
            rospy.loginfo('[qr_detector] 一帧里有 %d 张码，选中 %r（其余 %s）',
                          len(results), best[0],
                          [r[0] for r in results if r is not best])
        return best if best is not None else (None, None)

    def _decode(self, gray):
        """对单帧灰度图做多尺度解码，返回 (payload, points) 或 (None, None)。"""
        results = self._decode_backend(gray)
        if results:
            return self._pick(results, gray.shape)

        # 上采样二次尝试：应对远距离小码
        if self.upscale and self.upscale > 1.0:
            big = cv2.resize(gray, None, fx=self.upscale, fy=self.upscale,
                             interpolation=cv2.INTER_CUBIC)
            results = self._decode_backend(big)
            if results:
                for i, (pl, pt) in enumerate(results):
                    if pt is not None:
                        results[i] = (pl, pt / self.upscale)
                return self._pick(results, gray.shape)

        # 反色尝试：白底黑码 / 黑底白码都可能
        inv = cv2.bitwise_not(gray)
        results = self._decode_backend(inv)
        if results:
            return self._pick(results, gray.shape)
        return None, None

    @staticmethod
    def _decode_backend(gray):
        out = []
        if HAS_PYZBAR:
            for sym in pyzbar.decode(gray):
                try:
                    payload = sym.data.decode('utf-8')
                except UnicodeDecodeError:
                    payload = sym.data.decode('gbk', 'replace')
                pts = np.array([[p.x, p.y] for p in sym.polygon], dtype=np.float32) \
                    if getattr(sym, 'polygon', None) else None
                out.append((payload, pts))
        elif HAS_ZBAR:
            scanner = zbar.ImageScanner()
            scanner.parse_config('enable')
            img = zbar.Image(gray.shape[1], gray.shape[0], 'Y800', gray.tostring())
            scanner.scan(img)
            for sym in img:
                out.append((sym.data.decode('utf-8', 'replace'), None))
        return out

    def _guess_center_depth(self, points):
        """用深度图估算二维码中心距离（米），失败返回 None。"""
        if points is None or self._depth_frame is None:
            return None
        try:
            h, w = self._depth_frame.shape[:2]
            cx = int(np.clip(points[:, 0].mean(), 0, w - 1))
            cy = int(np.clip(points[:, 1].mean(), 0, h - 1))
            patch = self._depth_frame[max(0, cy - 5):cy + 6, max(0, cx - 5):cx + 6]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size == 0:
                return None
            d = float(np.median(valid))
            # Astra 深度图单位通常为 mm
            if d > 100.0:
                d = d / 1000.0
            return d
        except Exception:
            return None

    # -- 回调 ---------------------------------------------------------------
    def on_depth(self, msg):
        try:
            self._depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            rospy.logwarn_throttle(5.0, '[qr_detector] 深度图转换失败: %s', exc)

    def on_image(self, msg):
        with self.lock:
            if self._done:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:
                rospy.logwarn_throttle(5.0, '[qr_detector] 图像转换失败: %s', exc)
                return

            self._decode_attempts += 1
            h, w = frame.shape[:2]

            # ROI：只看下半部分（二维码贴在低处）
            y0 = int(h * self.roi_top_ratio)
            roi = frame[y0:, :]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            if self.enable_clahe:
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                gray = clahe.apply(gray)

            payload, points = self._decode(gray)
            debug = frame.copy()

            if payload:
                if points is not None:
                    pts = points.copy()
                    pts[:, 1] += y0
                    cv2.polylines(debug, [pts.astype(np.int32)], True, (0, 255, 0), 2)
                    self._last_distance = self._guess_center_depth(pts)

                # 去抖
                if payload == self._last_payload:
                    self._payload_count += 1
                else:
                    self._last_payload = payload
                    self._payload_count = 1
                    order, reason = parse_room_order(payload)
                    rospy.loginfo('[qr_detector] 新载荷 %r -> 解析 %s (%s)',
                                  payload, order, reason)

                if self._payload_count >= self.confirm_frames:
                    order, reason = parse_room_order(payload)
                    if order:
                        self._done = True
                        self._order = order
                        self._raw_payload = payload
                        self._publish_result(order, payload, reason)
                    else:
                        # 载荷稳定但解析失败：播报原文，便于人工兜底
                        self._payload_count = 0
                        self._set_status(u'识别到二维码但无法解析：%s（原因：%s）'
                                         % (payload, reason))

            # 距离/进度信息
            info = 'attempt=%d payload=%s dist=%s' % (
                self._decode_attempts,
                (payload or '-'),
                ('%.2fm' % self._last_distance) if self._last_distance else '-')
            cv2.putText(debug, _u8(info),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if self.pub_debug is not None:
                try:
                    self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
                except Exception:
                    pass

    # -- 服务 ---------------------------------------------------------------
    def on_detect_once(self, _req):
        """阻塞式识别一次，最多等待 15 秒。"""
        if self.pub_light is not None:
            self.pub_light.publish(String(data='on'))
            rospy.sleep(0.3)

        with self.lock:
            self._done = False
            self._payload_count = 0
            self._last_payload = None
            self._decode_attempts = 0

        deadline = time.time() + 15.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                if self._done:
                    return TriggerResponse(
                        success=True,
                        message=_u8(u'%s|%s' % (','.join(str(x) for x in self._order),
                                                self._raw_payload or u'')))
            rate.sleep()

        self._set_status(u'二维码识别超时（%d 帧尝试）' % self._decode_attempts)
        return TriggerResponse(success=False, message=_u8(u'识别超时'))

    def on_force_order(self, req):
        """人工兜底：请求 message 形如 "2,3,4" 强制注入顺序。"""
        raw = getattr(req, 'message', '') or ''
        order, reason = parse_room_order(raw)
        if not order:
            return TriggerResponse(success=False, message=_u8(u'无法解析：%s' % reason))
        forced = u'FORCED:' + _to_text(raw)
        with self.lock:
            self._done = True
            self._order = order
            self._raw_payload = forced
        self._publish_result(order, _u8(forced), u'人工注入')
        return TriggerResponse(success=True, message=_u8(u'已注入顺序 %s' % order))


if __name__ == '__main__':
    setup_stdout()
    fix_ros_logging()
    rospy.init_node('qr_detector')
    try:
        node = QrDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
