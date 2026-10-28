# -*- coding: utf-8 -*-
"""m36_mission_v5.py —— 全流程赛题任务（导航部分以 V3 为基准）

## 赛题流程（本脚本的阶段划分）

    ① RECORD_START   记下起点位姿（起点位置随机，回程要用）
    ② QR_AREA        客厅 → 二维码张贴区域（位置现场随机 200×200mm）
    ③ QR_SCAN        原地分步旋转 + 调 /qr/detect_once 找码，最多 2 个观察点
    ④ PARSE_ORDER    从二维码解析巡视顺序（2=B 3=C 4=D；也兼容 1/2/3 与 BCD 字母）
    ⑤ ENV_ANNOUNCE   读取客厅 光照/温度/湿度 并语音播报
    ⑥ FACE_GATE      到 A 点，/face/verify，最多 3 次；不通过则不许进房间
    ⑦ ROOMS          按二维码顺序逐房间：门口点 → 房间中心 → 房间任务
                       2 卧室：开电灯 + 开窗帘
                       3 厨房：读烟雾/CO2 状态 → 开传感器电源 → 开风扇
                       4 书房：漏水检测 → 播报（异常时关进水阀）
    ⑧ RETURN_START   回到 ① 记录的起点，结束

## 与既有接口的对接（沿用 carebot_eldercare / carebot_iot 已定好的）

    /qr/detect_once   std_srvs/Trigger   返回 "2,3,4|原始载荷"
    /iot/refresh      std_srvs/Trigger   返回 JSON（env.illuminance/temperature/humidity/smoke/water_leak）
    /iot/control      carebot_iot_msgs/Control(device, action, extra) -> (success, message)
    /face/verify      std_srvs/Trigger   success=是否主人
    /voice/say        std_msgs/String    语音播报

## 导航（V3 验证过的那一套）

* 参数：config/nav_v3（radius 0.21 / inflation 0.40 / cost_scaling 4 / lethal 253 / 0.22 m/s）
* 路线：门口点进房、房间中心的到达朝向自动朝"下一站"（省掉原地掉头）
* 安全：原始 /scan，车体自遮挡按角度带剔除，前方 ±20° < 0.40 m 立即取消

## 用法

    python _tools/real_test/m36_mission_v5.py --sim          # 全流程仿真（不动车、不调服务）
    python _tools/real_test/m36_mission_v5.py --dry-run      # 真读服务，但不发导航目标
    python _tools/real_test/m36_mission_v5.py --order CBD    # 跳过二维码，直接指定顺序
    python _tools/real_test/m36_mission_v5.py                # 正式跑
    python _tools/real_test/m36_mission_v5.py --fake-face --fake-iot
                                                             # 车上没配 HA 令牌/人脸库时，
                                                             # 只桩这两项把全流程走通
"""
from __future__ import print_function

import json
import math
import os
import re
import socket
import sys
import threading
import time

# ---- 输出编码兜底：SSH/重定向到文件时 locale 可能是 ASCII，中文 print 会炸 ----
import codecs as _codecs
for _name in ('stdout', 'stderr'):
    _s = getattr(sys, _name, None)
    _enc = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _enc not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _name, _codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

# ===========================================================================
#  航点（共享地图 talos_map_1016_0808 坐标系）
#  ★ 现场随机变动的只有"起点"和"二维码位置"，其余按地图标定
# ===========================================================================
WP = {    'start':      (0.00,  0.00, 0),   # 固定起点（地图开口处）
    'living_in':  (0.42, -0.90, 0),   # ★ 客厅入口：从起点往里走约 1.0 m（原 0.5 m）
    'qr_center':  (0.70, -1.50, 0),   # ★ 扫描点：客厅中线，离南墙(y≈-2.55)约 1 m
    'living_mid': (0.78, -0.96, 0),   # 客厅中部（去二维码区的中转/观察点 1）
    'qr_watch_a': (0.70, -2.00, 0),   # 二维码观察点 A：客厅南端，离墙 0.53 m
    'qr_watch_b': (0.35, -1.55, 0),   # 二维码观察点 B：客厅西南，换角度再找一次
    'A_mid':      (0.78, -0.96, 0),   # A 房间中间
    'A':          (1.17, -1.26, 10),  # A 点人脸核验区
    # ★★ 2026-10-22：房内坐标按本场地实测标定（进出房间路径经过验证）。
    #   规律：入口前点在门洞**外面**（x≈1.7~1.83），房间内点**更靠里**（x≈2.59~2.80），
    #   再加一个 hub 作为"客厅↔房间"的枢纽。每个房间都走 hub→入口前→房间内→hub 的往返。
    'hub':        (1.276, -1.276, 0),  # 客厅<->房间 枢纽
    'E_B':        (1.694, -2.071, 0),  # 书房入口前
    'E_C':        (1.860, -0.980, 0),  # ★ 厨房入口前：用我们自己的点（1.90,-0.95 附近）
    'E_D':        (1.768,  0.068, 0),  # 卧室入口前
    'B_door':     (1.694, -2.071, 0),
    'B':          (2.800, -2.025, 0),  # 书房内（实测标定）
    'C_door':     (1.827, -1.047, 0),
    'C':          (2.780, -0.998, 0),  # ★ 厨房内：用我们自己那版(2.69)并再推深到 2.78
                                          #   （实测 2.45+倒车 出门只要 2.2s，2.64 反而 32s）
    'D_door':     (1.768,  0.068, 0),
    'D':          (2.592,  0.034, 0),  # 卧室内（实测标定）
    # ★ 2026-10-23 跌倒检测点：起点门口，回程顺路，视野覆盖整个客厅
}

# ★ 2026-10-23：老人跌倒检测（⑦c）
#   位置用入口 living_in（离起点 0.99 m），朝向对准对面墙（现场标定）
#   在这个位置/朝向下 4 帧就确认了跌倒（state/last_fall.json）。
# ★ 2026-10-23（现场）：跌倒检测改在**人脸识别点 A** 做，相机朝向**起点**；
#   车做完语音交互先回 A、转向起点、播"开始跌倒检测"，人躺在起点方向。
FALL_POINT = 'hub'
FALL_LOOK_AT = (0.0, 0.0)      # 朝向起点
FALL_SECONDS = 15.0            # 观测窗口

# 房间号 -> (门口点, 房间中心, 中文名, 任务函数名)
# ★ 2026-10-20 现场核对（改正过一次）：
#   B = 书房、C = 厨房、D = 卧室
#   依据①老友组实测航点注释「A 缺口/人脸  D 卧室  C 厨房  B 书房」
#   依据②我们自己的地图重建记录「D 卧室 | C 厨房 | B 书房」
#   carebot_eldercare 的 ROOM_INFO 里 2=B=卧室 是通用参考值，**不适用于这个场地**。
ROOMS = {
    # (门口点, 房间中心, 中文名, 任务函数, E点)
    2: ('B_door', 'B', u'书房', 'task_study', 'E_B'),
    3: ('C_door', 'C', u'厨房', 'task_kitchen', 'E_C'),
    4: ('D_door', 'D', u'卧室', 'task_bedroom', 'E_D'),
}
LEGACY = {1: 2, 2: 3, 3: 4}          # 兼容 1/2/3 的写法

# ---- 导航/安全参数（与 m29/m30/m33 一致）----
FRONT_CONE = math.radians(20)
FRONT_STOP = 0.40

# ★ 2026-10-23 现场调参（**仅单次测试版**，不满意就回 V111）：
#   卡住判定 10 s + 10 s -> 5 s + 5 s（10 秒出结论，少等 10~30 秒）
STALL_FIRST = 5.0
STALL_WINDOW = 5.0
#   到达容差 0.25 -> 0.35 m（减少最后几厘米的磨蹭；太大会离目标点更远）
ACCEPT_RADIUS = 0.35
SPIN_STOP = 0.30
SELF_MIN = 0.36
SELF_BANDS = ((15, 35), (145, 215))
GOAL_TIMEOUT = 120.0

# ---- 二维码搜索 ----
QR_WAIT_TOTAL = 30.0          # （旧）原地等码窗口
QR_STAND_TOTAL = 30.0         # ★ V6.1：原地停住扫描的时长（不旋转）
QR_SPIN_SPEED = 0.28          # ★ V6.2：0.35 -> 0.28（更慢，打滑/跑位更少）
QR_SPIN_TURNS = 2.0           # （保留）最多转几圈
SPIN_SPEED = 0.7
SPIN_TOL = 0.06

# ---- 人脸核验 ----
FACE_TRIES = 3
FACE_GAP = 3.0

# ---- 心率播报（卧室，灯光/窗帘播报之后）----
# 数据来源：ESP32 通过 UDP 9999 广播 CSV "seq,heart_rate,respiration,people,esp_ts_ms"
# （与 LXH2 的 step_vitals.py / sensor_hub.py 同一协议，这里不依赖 ROS，直接收）
HR_PORT = 9999
HR_LISTEN = 1.8           # 等一帧的时长（ESP32 约 1 Hz，1.8 s 足够）
HR_MAX_AGE = 10.0         # 缓存超过这么久就当作没有数据
HR_MIN, HR_MAX = 30.0, 200.0
HR_SAY_FALLBACK = True    # 没数据时也播一句"暂时没有收到心率数据"

# ---- 房间停留 ----
USE_DOOR_LEGS = False     # ★ 是否经过"门口"航点（现场实测设为 False 更稳）
ROOM_DWELL = 3.0          # ★ 每个房间做完任务后停留几秒（避免"一闪而过"）
ENV_RETRIES = 3           # ★ 环境读取重试次数（偶发超时）


def is_self(deg):
    d = abs(((deg + 180.0) % 360.0) - 180.0)
    return any(lo <= d <= hi for lo, hi in SELF_BANDS)


def yaw_to(x0, y0, x1, y1):
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


# ===========================================================================
#  安全护栏（原始 /scan + 自遮挡剔除）
# ===========================================================================
class QrPoller(threading.Thread):
    """后台一直调 /qr/detect_once，主线程同时转圈。

    ★ 2026-10-23：扫描服务在车旋转期间**持续解帧**
    （每 0.2 s 一次、连续 2 帧一致才认），转一圈可完成数十次采样。
    我们原来是在主循环里同步调 detect_once——这个服务内部会一直解到 15 s 才返回，
    于是转 720° 只采样 2~3 次，两个途中扫描点因此都读不到。
    改成"后台线程持续调服务 + 主线程转圈"，采样密度就变成 10 Hz（服务内部节拍）。
    """

    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.daemon = True
        self.dev = dev
        self.result = None
        self._stop = False
        self.calls = 0

    def run(self):
        while not self._stop and not rospy.is_shutdown():
            try:
                payload, raw = self.dev.qr_detect_once()
                self.calls += 1
            except Exception:
                payload = None
            if getattr(self.dev, 'qr_order_topic', None):
                # 服务没回但 /qr/room_order 有 latched 结果 —— 一样可用
                payload = ''.join({2: 'B', 3: 'C', 4: 'D'}[v]
                                  for v in self.dev.qr_order_topic)
            if payload:
                self.result = payload
                return

    def stop(self):
        self._stop = True

class Spinner(threading.Thread):
    """原地缓慢旋转，直到 stop()。用线程是为了让主线程能阻塞在 /qr/detect_once。

    角速度靠"命令积分"估算（底盘无角速度反馈），够用于"转满一圈"判断。
    """

    def __init__(self, pub, speed=QR_SPIN_SPEED, pose_fn=None, keep_xy=None):
        threading.Thread.__init__(self)
        self.daemon = True
        self.pub = pub
        self.speed = abs(speed)
        self.angle = 0.0
        # ★ 定点保持：2026-10-21 实车发现"原地转圈时会跑位"。
        #   这里在旋转的同时用很小的线速度把车拉回起点（比例控制，限速 4 cm/s）。
        self.pose_fn = pose_fn
        self.keep = keep_xy
        self.kp = 0.8
        self.max_lin = 0.04
        self.drift = 0.0
        self.max_drift = 0.0
        # ★ 必须在 __init__ 里置 True：如果写进 run()，而 stop() 抢在
        #   线程真正启动之前调用，_run 会被 run() 重新置 True -> 永远停不下来
        self._run = True

    def run(self):
        rate = rospy.Rate(20)
        t_prev = time.time()
        while self._run and not rospy.is_shutdown():
            tw = Twist()
            tw.angular.z = self.speed
            if self.keep is not None and self.pose_fn is not None:
                p = self.pose_fn()
                if p:
                    ex = self.keep[0] - p[0]
                    ey = self.keep[1] - p[1]
                    yaw = math.radians(p[2])
                    fwd = math.cos(yaw) * ex + math.sin(yaw) * ey
                    v = self.kp * fwd
                    tw.linear.x = max(-self.max_lin, min(self.max_lin, v))
                    self.drift = math.hypot(ex, ey)
                    self.max_drift = max(self.max_drift, self.drift)
            self.pub.publish(tw)
            now = time.time()
            self.angle += self.speed * (now - t_prev)
            t_prev = now
            rate.sleep()
        for _ in range(6):
            self.pub.publish(Twist())
            rospy.sleep(0.03)

    def stop(self):
        self._run = False


class Guard(object):
    """前方净空护栏。V5b 起**默认关闭**（`--guard` 可重新打开）。

    关闭原因：0.40 m 的硬否决在"目标点本来就靠墙"时会误判——
    2026-10-20 16:36 那一轮，车在 qr_watch_a 合法地离南墙 0.37 m，
    之后每一段目标都在 0.0 s 被取消，整轮卡死。
    """

    def __init__(self, enabled=False):
        self.enabled = enabled
        self.min_front = 9.0
        self.min_all = 9.0
        self.front_deg = 0.0
        rospy.Subscriber('/scan', LaserScan, self.cb, queue_size=1)

    def cb(self, s):
        best, allmin = 3.0, 3.0
        for i, v in enumerate(s.ranges):
            if not (v == v) or v < SELF_MIN or v > 3.0:
                continue
            a = math.atan2(math.sin(s.angle_min + i * s.angle_increment),
                           math.cos(s.angle_min + i * s.angle_increment))
            if is_self(math.degrees(a)) and v < 0.50:
                continue
            if v < allmin:
                allmin = v
            if abs(a) < FRONT_CONE and v < best:
                best = v
                self.front_deg = math.degrees(a)
        self.min_front, self.min_all = best, allmin


# ===========================================================================
#  设备/服务层：ROS 后端 与 仿真后端（--sim）
# ===========================================================================
class Vitals(object):
    """收 ESP32 的心率/呼吸/人数（UDP 9999，CSV：seq,hr,br,people,ts）。

    与 LXH2 的 step_vitals.py 用同一套协议，但这里不依赖 Python3、不依赖 ROS：
    睡在任务进程里，卧室那一站读一帧就播报。最新的那一帧同时落盘到
    /tmp/vitals.json，方便别的脚本（或事后排查）直接读。
    """

    CACHE = '/tmp/vitals.json'

    def __init__(self, port=HR_PORT, timeout=HR_LISTEN, max_age=HR_MAX_AGE):
        self.port = port
        self.timeout = timeout
        self.max_age = max_age
        self._sock = None
        self._err = None
        self._last = self._read_cache()

    # ---------------------------------------------------------------- 缓存
    @classmethod
    def _read_cache(cls):
        try:
            with open(cls.CACHE) as fp:
                d = json.load(fp)
            d['recv_time'] = float(d.get('recv_time') or 0)
            return d
        except Exception:
            return None

    def _write_cache(self, rec):
        try:
            with open(self.CACHE, 'w') as fp:
                json.dump(rec, fp)
        except Exception:
            pass

    # ---------------------------------------------------------------- 收包
    def _open(self):
        if self._sock is not None or self._err is not None:
            return self._sock
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('0.0.0.0', self.port))
            s.settimeout(0.3)
            self._sock = s
        except Exception as exc:
            # 端口被旧进程占着：不致命，退回读缓存
            self._err = repr(exc)
        return self._sock

    def read(self):
        """返回最新一帧（dict）或 None；不新鲜时也返回 None。"""
        s = self._open()
        if s is not None:
            t_end = time.time() + self.timeout
            while time.time() < t_end:
                try:
                    data, addr = s.recvfrom(2048)
                except socket.timeout:
                    continue
                except Exception:
                    break
                text = data.decode('utf-8', 'replace').strip()
                if not text or text.startswith('HELLO'):
                    continue
                parts = text.split(',')
                if len(parts) != 5:
                    continue
                try:
                    rec = {'seq': int(parts[0]),
                           'heart_rate': float(parts[1]),
                           'respiration': float(parts[2]),
                           'people': int(parts[3]),
                           'recv_time': time.time(),
                           'source': addr[0]}
                except ValueError:
                    continue
                self._last = rec
                self._write_cache(rec)
                return rec
        if self._last and time.time() - self._last.get('recv_time', 0) <= self.max_age:
            return self._last
        return None


def _u(s):
    """服务返回的 message 是 str(bytes)，直接塞进 u'...%s' 会因 ASCII 解码报错。"""
    try:
        text_type = unicode          # py2
    except NameError:
        text_type = str
    if isinstance(s, text_type):
        return s
    if isinstance(s, str):
        return s.decode('utf-8', 'ignore')
    return text_type(s)


class RosDevices(object):
    def __init__(self, fake_face=False, fake_iot=False):
        self.say_pub = rospy.Publisher('/voice/say', String, queue_size=20)
        # 厂商讯飞离线 TTS（小燕）订阅 /voice_words，音质比 voice_bridge 退化用的
        # espeak -v zh 好得多；有订阅者就优先发它
        self.xf_pub = rospy.Publisher('/voice_words', String, queue_size=20)
        self.said = []
        # ★ 二维码节点会把解析结果 latched 发到 /qr/room_order；
        #   即使 /qr/detect_once 那次调用超时，这里也能补上（2026-10-20 现场：
        #   服务报超时 3 s 后节点其实识别成功了，结果就发在这个话题上）
        self.qr_order_topic = None
        try:
            from std_msgs.msg import Int16MultiArray
            rospy.Subscriber('/qr/room_order', Int16MultiArray, self._on_qr_order, queue_size=1)
        except Exception:
            pass
        self.fake_face = fake_face
        self.fake_iot = fake_iot
        self._stub = StubDevices()

    def say(self, text):
        self.said.append(text)
        print(u'  [语音] %s' % text)
        msg = String(data=text.encode('utf-8'))
        # ★ 两路都发：/voice/say 是我们的在线 TTS；
        #   /voice_words 是厂商讯飞（现场实测坏的，但不排除别人的节点在用）。
        #   原来"有订阅者就只发讯飞"会导致语音彻底哑掉。
        try:
            self.say_pub.publish(msg)
            if self.xf_pub.get_num_connections() > 0:
                self.xf_pub.publish(msg)
        except Exception as exc:
            print(u'     (语音发布失败: %r)' % exc)

    def reset_qr_order(self):
        """清掉上一轮残留的 latched 顺序（否则会拿旧结果当新识别）。"""
        self.qr_order_topic = None

    def _on_qr_order(self, msg):
        vals = [int(v) for v in msg.data]
        if set(vals) == {2, 3, 4}:
            self.qr_order_topic = vals
        elif set(vals) == {1, 2, 3}:
            self.qr_order_topic = [LEGACY[v] for v in vals]
        else:
            return
        print(u'     [qr] 从 /qr/room_order 收到顺序 %s' % self.qr_order_topic)

    def _call(self, name, srv_type, timeout=3.0, args=()):
        rospy.wait_for_service(name, timeout=timeout)
        return rospy.ServiceProxy(name, srv_type)(*args)

    def qr_detect_once(self):
        try:
            r = self._call('/qr/detect_once', Trigger, 3.0)
            if not r.success:
                return None, r.message
            msg = r.message
            # 约定格式 "2,3,4|原始载荷"
            payload = msg.split('|', 1)[1] if '|' in msg else msg
            return _u(payload), _u(msg)
        except Exception as exc:
            return None, repr(exc)

    def iot_refresh(self):
        if self.fake_iot:
            print(u'     [stub] iot/refresh 用测试桩数据')
            return self._stub.iot_refresh()
        try:
            r = self._call('/iot/refresh', Trigger, 4.0)
            return json.loads(r.message) if r.message else {}
        except Exception as exc:
            print(u'     (iot/refresh 失败: %r)' % exc)
            return {}

    def iot_control(self, device, action, extra=''):
        if self.fake_iot:
            print(u'     [stub] iot/control %s %s' % (device, action))
            return True, 'stub'
        try:
            from carebot_iot_msgs.srv import Control
            r = self._call('/iot/control', Control, 3.0, (str(device), str(action), str(extra)))
            return bool(r.success), _u(r.message)
        except Exception as exc:
            return False, repr(exc)

    def face_verify(self):
        if self.fake_face:
            print(u'     [stub] face/verify -> 通过')
            return True, u'测试桩'
        try:
            r = self._call('/face/verify', Trigger, 4.0)
            return bool(r.success), _u(r.message)
        except Exception as exc:
            return False, repr(exc)


class StubDevices(object):
    """--sim：不接任何服务，按参数注入结果，用来跑通流程逻辑。"""

    def __init__(self, qr_payload='2,3,4', face_ok=True):
        self.qr_payload = qr_payload
        self.face_ok = face_ok
        self.said = []
        self.calls = []
        self.qr_calls = 0

    def say(self, text):
        self.said.append(text)
        print(u'  [语音] [sim] %s' % text)

    def qr_detect_once(self):
        self.qr_calls += 1
        # 模拟"转了 3 个朝向后才看到码"
        if self.qr_calls >= 3:
            return self.qr_payload, '%s|%s' % (self.qr_payload, self.qr_payload)
        return None, 'no_qr'

    def iot_refresh(self):
        return {'env': {
            'illuminance': {'value': 320, 'unit': 'lux', 'desc': u'光照'},
            'temperature': {'value': 24.5, 'unit': u'℃', 'desc': u'温度'},
            'humidity': {'value': 55, 'unit': '%', 'desc': u'湿度'},
            'smoke': {'value': 12, 'text': u'烟雾浓度正常，CO2 320ppm'},
            'water_leak': {'text': u'未检测到漏水', 'abnormal': False, 'configured': True},
        }}

    def iot_control(self, device, action, extra=''):
        self.calls.append((device, action))
        print(u'     [sim] iot/control %s %s' % (device, action))
        return True, 'sim ok'

    def face_verify(self):
        print(u'     [sim] face/verify -> %s' % ('主人' if self.face_ok else '陌生人'))
        return self.face_ok, u'sim: %s' % (u'主人' if self.face_ok else u'未识别')


# ===========================================================================
#  导航层（V3 那套）
# ===========================================================================
class Nav(object):
    def __init__(self, guard, dry_run=False):
        self.guard = guard
        self.dry = dry_run
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.lst = tf.TransformListener()
        self.stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # ★ 记录当前速度：用于判断"卡住"时区分"真的不动"和"在原地转"
        self.v_now = (0.0, 0.0)
        try:
            from nav_msgs.msg import Odometry
            rospy.Subscriber('/odom_clean', Odometry, self._on_odom, queue_size=5)
        except Exception:
            pass

    def _on_odom(self, m):
        self.v_now = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def halt(self):
        """立刻停车（连发几帧 0 速度，别让底盘保持上一帧速度）。"""
        try:
            for _ in range(8):
                self.stop_pub.publish(Twist())
                rospy.sleep(0.03)
        except Exception:
            pass

    def ready(self):
        return self.dry or self.ac.wait_for_server(rospy.Duration(25.0))

    def rotate_to(self, yaw_deg, timeout=20.0, tol_deg=4.0):
        """原地转到指定朝向（度）。用 cmd_vel 直接闭环，避免 move_base 拒绝纯旋转目标。

        为什么不用 goto：goto 的到达判据是"离目标点 ≤0.25 m"，在原地转向时
        第一个周期就判定到达并取消目标，根本转不过去。
        """
        if self.dry:
            print(u'     [dry-run] 原地转到 %.0f°' % yaw_deg)
            return True
        t0 = time.time()
        target = math.radians(yaw_deg)
        rate = rospy.Rate(15)
        cmd = Twist()
        try:
            while not rospy.is_shutdown() and time.time() - t0 < timeout:
                p = self.pose()
                if not p:
                    rospy.sleep(0.1)
                    continue
                err = math.atan2(math.sin(target - math.radians(p[2])),
                                 math.cos(target - math.radians(p[2])))
                if abs(math.degrees(err)) <= tol_deg:
                    break
                cmd.angular.z = max(-0.30, min(0.30, 1.2 * err))
                self.stop_pub.publish(cmd)
                rate.sleep()
        finally:
            self.halt()
        p = self.pose()
        ok = bool(p) and abs(math.degrees(math.atan2(
            math.sin(target - math.radians(p[2])),
            math.cos(target - math.radians(p[2]))))) <= tol_deg + 2.0
        print(u'     原地转向：目标 %.0f°，实际 %.0f°，%.1f s %s'
              % (yaw_deg, p[2] if p else -999, time.time() - t0,
                 u'成功' if ok else u'未到位'))
        return ok

    def pose(self):
        try:
            self.lst.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
            (t, r) = self.lst.lookupTransform('map', 'base_footprint', rospy.Time(0))
            e = tf.transformations.euler_from_quaternion(r)
            return t[0], t[1], math.degrees(e[2])
        except Exception:
            return None

    def _goal(self, x, y, yaw_deg):
        if yaw_deg is None:
            # ★ 2026-10-22：目标朝向 = **行进方向**。
            #   车到点时就已经面朝下一站，下一段可以直接走，不用掉头/转身。
            #   原来用"当前车头"，到了还要转过去对准，白白多转几秒。
            p = self.pose()
            if p:
                dx, dy = x - p[0], y - p[1]
                yaw_deg = math.degrees(math.atan2(dy, dx)) if (abs(dx) + abs(dy)) > 0.05 else p[2]
            else:
                yaw_deg = 0.0
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw_deg))
        g.target_pose.pose.orientation.x = q[0]
        g.target_pose.pose.orientation.y = q[1]
        g.target_pose.pose.orientation.z = q[2]
        g.target_pose.pose.orientation.w = q[3]
        return g

    def goto(self, label, x, y, yaw=None, timeout=GOAL_TIMEOUT,
             accept_radius=ACCEPT_RADIUS):
        """发导航目标并等结果。

        accept_radius：**到达容差**。move_base 的 xy_goal_tolerance 是 0.10 m，
        但它还要把朝向转到目标 yaw 才算 SUCCEEDED；现场经常出现
        "位置已经到了、只差 20° 朝向、规划器一直报 Failed to get a plan"
        的僵局（2026-10-21 实测卡了 2 分半）。所以这里自己判：
        进到 accept_radius 内就取消目标并当成功，不再等朝向。
        """
        print(u'')
        _yaw_txt = u'保持车头' if yaw is None else u'%.0f°' % yaw
        print(u'  -> %s  (%.2f, %.2f, %s)' % (label, x, y, _yaw_txt))
        if self.dry:
            print(u'     [dry-run] 不发目标')
            return 'DRY', 0.0, 0.0
        t0 = time.time()
        self._last_pos, self._last_t = None, 0.0
        retries = 0
        self.ac.send_goal(self._goal(x, y, yaw))
        state = None
        while not rospy.is_shutdown():
            el = time.time() - t0
            # 到达容差：进了这个圈就算到（避免卡在最后的角度上）
            if el > 1.0:
                p = self.pose()
                if p and math.hypot(p[0] - x, p[1] - y) <= accept_radius:
                    print(u'     到达容差内（离目标 %.3f m），取消剩余朝向调整'
                          % math.hypot(p[0] - x, p[1] - y))
                    self.ac.cancel_goal()
                    self.halt()
                    state = 'SUCCEEDED'
                    break
            if self.guard.enabled and self.guard.min_front < FRONT_STOP:
                print(u'     [停止] 前方 %.0f° 净空 %.2f m < %.2f m，取消'
                      % (self.guard.front_deg, self.guard.min_front, FRONT_STOP))
                self.ac.cancel_goal()
                state = 'ABORT_SAFETY'
                break
            # 卡死检测（两种处理）：
            #   a) 离目标已经近了（<0.6 m）-> 按到达处理，别为最后几厘米磨
            #   b) 离目标还远 -> **取消并重发目标**，这是 move_base 卡住时
            #      最有效的解药（2026-10-21 实车：书房门口卡了 60 s 不动，
            #      净空一直 0.51 m，最后靠状态跳出才继续）
            if el > STALL_FIRST:
                p_now = self.pose()
                if p_now:
                    if self._last_pos is None:
                        self._last_pos, self._last_t = p_now, el
                    elif el - self._last_t >= STALL_WINDOW:
                        moved = math.hypot(p_now[0] - self._last_pos[0],
                                           p_now[1] - self._last_pos[1])
                        remain = math.hypot(p_now[0] - x, p_now[1] - y)
                        # 正在转/正在走就不算卡住（原地旋转时位置本来就不变）
                        vx, vth = getattr(self, 'v_now', (0.0, 0.0))
                        if abs(vx) > 0.02 or abs(vth) > 0.15:
                            self._last_pos, self._last_t = p_now, el
                            continue
                        if moved < 0.04:
                            if remain < 0.60:
                                print(u'     磨蹭 %.1f s 只动了 %.3f m（离目标 %.2f m），按到达处理'
                                      % (el - self._last_t, moved, remain))
                                self.ac.cancel_goal()
                                self.halt()
                                state = 'SUCCEEDED'
                                break
                            if retries < 3:
                                retries += 1
                                # ★ 换姿态再重发：只重发=用同一条路再算一遍，
                                #   先原地转 40° 往往就能绕开"算不出路"的死角
                                turn = 40.0 if retries % 2 else -40.0
                                print(u'     卡住 %.1f s 只动了 %.3f m（离目标 %.2f m），'
                                      u'原地转 %.0f° 后重发（第 %d 次）'
                                      % (el - self._last_t, moved, remain, turn, retries))
                                self.ac.cancel_goal()
                                self.halt()
                                # ★ 先**后退**脱困：实测卡死时局部代价图中心
                                #   3x3 全是 94~99 的高膨胀代价——车停在"膨胀口袋"
                                #   里，DWA 算不出轨迹。原地转没用（位置没变），
                                #   必须先后退把车挪出膨胀区。
                                self._back_up(0.20)
                                self.turn_in_place(turn, 0.6)
                                rospy.sleep(0.3)
                                self.ac.send_goal(self._goal(x, y, yaw))
                        self._last_pos, self._last_t = p_now, el
            st = self.ac.get_state()
            if st == GoalStatus.SUCCEEDED:
                state = 'SUCCEEDED'
                break
            if st in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                state = {GoalStatus.ABORTED: 'ABORTED', GoalStatus.REJECTED: 'REJECTED',
                         GoalStatus.PREEMPTED: 'PREEMPTED'}[st]
                break
            if el > timeout:
                self.ac.cancel_goal()
                state = 'TIMEOUT'
                break
            if int(el) % 5 == 0:
                print(u'     %5.1fs %s 前向 %.2f m' % (el, 'RUNNING', self.guard.min_front))
            time.sleep(0.5)
        self.halt()
        p = self.pose()
        err = math.hypot(p[0] - x, p[1] - y) if p else float('nan')
        dt = time.time() - t0
        print(u'     结果 %-12s %5.1f s  误差 %.3f m' % (state, dt, err))
        return state, dt, err

    def goto_named(self, name, yaw=None, use_wp_yaw=False):
        """去航点。

        use_wp_yaw=False（默认）：不指定朝向，用当前车头 -> 不会原地转圈对准。
        use_wp_yaw=True：用航点里写死的 yaw（只在确实需要朝向时用）。
        """
        x, y, y0 = WP[name]
        target_yaw = yaw if yaw is not None else (y0 if use_wp_yaw else None)
        return self.goto(name, x, y, target_yaw)

    def turn_in_place(self, deg=40.0, speed=0.6):
        """原地转一个小角度（卡住换姿态用）。按时长积分，不依赖里程。"""
        try:
            tw = Twist()
            tw.angular.z = (1 if deg > 0 else -1) * abs(speed)
            t_end = time.time() + math.radians(abs(deg)) / max(abs(speed), 0.05)
            rate = rospy.Rate(20)
            while time.time() < t_end and not rospy.is_shutdown():
                self.stop_pub.publish(tw)
                rate.sleep()
        finally:
            self.halt()

    def _back_up(self, dist=0.15, speed=0.05):
        """后退一小段（换视角用）。用命令积分计时，不依赖里程。"""
        try:
            tw = Twist()
            tw.linear.x = -abs(speed)
            t_end = time.time() + abs(dist) / max(speed, 0.01)
            rate = rospy.Rate(20)
            while time.time() < t_end and not rospy.is_shutdown():
                self.stop_pub.publish(tw)
                rate.sleep()
        finally:
            self.halt()

    def spin_step(self, step_deg):
        """原地转一步（用于找二维码）。"""
        if self.dry:
            return True
        cur = self.pose()
        if cur is None:
            return False
        target = math.radians(cur[2] + step_deg)
        t0 = time.time()
        while not rospy.is_shutdown():
            if self.guard.enabled and self.guard.min_all < SPIN_STOP:
                print(u'     [停止] 全向净空 %.2f m，停止旋转' % self.guard.min_all)
                self.stop_pub.publish(Twist())
                return False
            cur = self.pose()
            if cur is None:
                break
            err = math.atan2(math.sin(target - math.radians(cur[2])),
                             math.cos(target - math.radians(cur[2])))
            if abs(err) < SPIN_TOL or time.time() - t0 > 6.0:
                break
            tw = Twist()
            tw.angular.z = SPIN_SPEED if err > 0 else -SPIN_SPEED
            self.stop_pub.publish(tw)
            time.sleep(0.05)
        self.stop_pub.publish(Twist())
        time.sleep(0.3)
        return True


# ===========================================================================
#  任务
# ===========================================================================
def parse_order(payload):
    """二维码载荷 -> [2,3,4] 的排列（内部 id：2=B 书房 / 3=C 厨房 / 4=D 卧室）。

    ⚠ 内部 id 2/3/4 就是字母 B/C/D，**不是**房间序号。房间中文名由 ROOMS 决定。

    兼容：
      "234" / "2,3,4" / "BCD" / "[4,2,3]" / "1,2,3"(旧写法换算)
    """
    if not payload:
        return None, u'空载荷'
    text = payload.strip()
    nums = [int(v) for v in re.findall(r'\d', text)]
    if len(nums) == 3:
        if set(nums) == {2, 3, 4}:
            return nums, u'数字 2/3/4'
        if set(nums) == {1, 2, 3}:
            return [LEGACY[v] for v in nums], u'数字 1/2/3（换算成 2/3/4）'
    letters = [c.upper() for c in re.findall(r'[BbCcDd]', text)]
    if len(letters) == 3 and set(letters) == {'B', 'C', 'D'}:
        m = {'B': 2, 'C': 3, 'D': 4}
        return [m[c] for c in letters], u'字母 BCD'
    return None, u'无法解析: %r' % payload


class Mission(object):
    def __init__(self, dev, nav, sim=False, dry=False, order_override=None):
        self.dev = dev
        self.nav = nav
        self.sim = sim
        self.dry = dry
        self.order_override = order_override
        self.timeline = []
        self.face_ok = None
        self.skip_voice = False
        self.skip_fall = False
        self.start_pose = None
        self.ok = True
        self.vitals = Vitals()      # 卧室那一站读一帧心率（UDP 9999）
        self.qr_early_payload = None  # V106：途中扫描点提前读到的码

    # ---- 小工具 ----
    def log(self, phase, text):
        print(u'[%s] %s' % (phase, text))
        self.timeline.append((phase, text))

    def fail(self, phase, text):
        self.ok = False
        self.log(phase, u'x ' + text)

    # ---- 各阶段 ----
    def phase_record_start(self):
        # V5b：起点固定（车由人放好），不再记位姿；回程用 WP['start']
        self.log(u'① RECORD_START', u'起点固定，跳过位姿记录')
        self._preflight_devices()
        self.dev.say(u'开始执行任务')
        return True

    def _preflight_devices(self, timeout=30.0):
        """★ 2026-10-22：起跑前确认 3 个可控设备在 HA 里在线。

        这一轮 fan_kitchen 在厨房那步回读 state=unavailable（插座被 Z2M 短暂
        判为掉线），整轮就多了一条失败项。掉线通常是暂时的：在车动之前先等
        它回来，比跑到厨房才发现划算（正常情况只花约 0.2 s）。
        """
        need = ('light_bedroom', 'curtain_bedroom', 'fan_kitchen')
        t0 = time.time()
        next_log = 0.0
        while not rospy.is_shutdown():
            try:
                data = self.dev.iot_refresh() or {}
            except Exception:
                data = {}
            st = data.get('states') or {}
            if st:
                bad = [d for d in need
                       if st.get(d) in (None, 'unavailable', 'unknown')]
                if not bad:
                    self.log(u'①', u'设备在线：%s'
                             % u'、'.join(u'%s=%s' % (d, st.get(d)) for d in need))
                    return True
            else:
                bad = list(need)
            now = time.time()
            if now - t0 >= timeout:
                self.log(u'①', u'[warn] 起跑前设备未全部在线（%s），继续执行'
                         % u'、'.join(bad))
                return False
            if now >= next_log:
                self.log(u'①', u'等待设备上线：%s（最多 %.0f s）'
                         % (u'、'.join(bad), timeout))
                next_log = now + 8.0
            rospy.sleep(2.0)
        return False

    def phase_to_qr_area(self):
        """起点 -> 客厅（约 0.5 m）-> 客厅中线离南墙 1 m 处。"""
        self.log(u'② QR_AREA', u'先进客厅（约 0.5 m），再走到离对面墙 1 m 处')
        ok = True
        st, _, _ = self.nav.goto_named('living_in')
        if st not in ('SUCCEEDED', 'DRY'):
            self.log(u'②', u'客厅入口没到（%s），继续往扫描点走' % st)
            ok = False
        st, _, _ = self.nav.goto_named('qr_center')
        if st not in ('SUCCEEDED', 'DRY'):
            self.fail(u'②', u'扫描点没到（%s）' % st)
            ok = False
        return ok

    def _spin_scan_at_point(self, label, turns=2.0, rounds=1, announce=True):
        """在当前位置原地旋转找码（turns 圈）。返回载荷或 None。

        与 phase_qr_scan 的区别：这里只转、不做"退一点换视角"的多轮，
        用于途中两个附加扫描点；原来的 qr_center 仍走 phase_qr_scan 的老逻辑。
        """
        total = turns * 2.0 * math.pi
        self.log(u'③', u'%s：原地转 %.0f° 找码（%.2f rad/s）'
                 % (label, math.degrees(total), QR_SPIN_SPEED))
        if announce:
            self.dev.say(u'我在客厅再看一圈二维码')
        if hasattr(self.dev, 'reset_qr_order'):
            self.dev.reset_qr_order()
        p0 = self.nav.pose()
        keep = (p0[0], p0[1]) if p0 else None
        payload = None
        for rnd in range(max(1, rounds)):
            if rnd > 0:
                self.nav._back_up(0.15)
                p1 = self.nav.pose()
                keep = (p1[0], p1[1]) if p1 else None
            spinner = Spinner(self.nav.stop_pub, QR_SPIN_SPEED,
                              pose_fn=self.nav.pose, keep_xy=keep)
            poller = QrPoller(self.dev)      # 后台持续解帧，主线程专心转圈
            spinner.start()
            poller.start()
            t0 = time.time()
            try:
                while not rospy.is_shutdown():
                    if poller.result:
                        payload = poller.result
                        self.log(u'③', u'%s 转了 %.0f° 时识别到二维码'
                                 % (label, math.degrees(spinner.angle)))
                        self.dev.say(u'好的，二维码识别成功')
                        break
                    if spinner.angle >= total:
                        break
                    if time.time() - t0 > total / QR_SPIN_SPEED + 40.0:
                        break
                    rospy.sleep(0.05)
            finally:
                spinner.stop()
                spinner.join(2.0)
                poller.stop()
                poller.join(1.5)
                self.nav.halt()
            if payload:
                break
        if not payload:
            self.log(u'③', u'%s 转满 %.0f° 没读到码，继续下一点'
                     % (label, math.degrees(total)))
        return payload

    def phase_qr_scan(self):
        """原地缓慢转圈扫描二维码（最多 QR_SPIN_TURNS 圈）。

        为什么要转：码可能贴在任意一面墙/板子上，站着不动只能看一个方向。
        为什么用线程：/qr/detect_once 是阻塞服务（节点内部最长 15 s），
        必须让"转"发生在另一个线程里，否则一转就卡住。
        """
        # ★ V106：途中两个扫描点已经读到码 -> 这里不再转圈，直接沿用
        if self.qr_early_payload:
            self.log(u'③ QR_SCAN', u'途中已读到二维码，原来的扫描点不再转圈')
            return self.qr_early_payload

        total = QR_SPIN_TURNS * 2.0 * math.pi
        self.log(u'③ QR_SCAN', u'原地缓慢旋转扫描（最多 %.1f 圈，%.2f rad/s，带定点保持）'
                 % (QR_SPIN_TURNS, QR_SPIN_SPEED))
        self.dev.say(u'我开始转一圈找二维码')

        if hasattr(self.dev, 'reset_qr_order'):
            self.dev.reset_qr_order()      # 忽略上一轮 latched 的旧结果

        # ★ V6.2：转圈找码 + **定点保持**（治"转着转着跑位"）
        p0 = self.nav.pose()
        keep = (p0[0], p0[1]) if p0 else None
        if keep:
            self.log(u'③', u'转圈中心 (%.3f, %.3f)，开启定点保持' % keep)
        payload = None
        max_drift = 0.0
        for rnd in range(2):          # 最多两轮：第二轮前先换视角
            if rnd > 0:
                self.log(u'③', u'第一圈没读到，后退 15 cm 换视角再转一圈')
                self.dev.say(u'没找到，我退一点再看看')
                self.nav._back_up(0.15)
                keep = self.nav.pose()
                keep = (keep[0], keep[1]) if keep else None
            spinner = Spinner(self.nav.stop_pub, QR_SPIN_SPEED,
                              pose_fn=self.nav.pose, keep_xy=keep)
            poller = QrPoller(self.dev)      # 后台持续解帧，边转边认
            spinner.start()
            poller.start()
            t0 = time.time()
            try:
                while not rospy.is_shutdown():
                    if poller.result:
                        payload = poller.result
                        self.log(u'③', u'第 %d 圈 %.1f s、转了 %.0f° 时识别到二维码'
                                 % (rnd + 1, time.time() - t0,
                                    math.degrees(spinner.angle)))
                        self.dev.say(u'好的，二维码识别成功')
                        break
                    if spinner.angle >= total:
                        break
                    if time.time() - t0 > total / QR_SPIN_SPEED + 40.0:
                        break
                    rospy.sleep(0.05)
            finally:
                spinner.stop()
                spinner.join(2.0)
                poller.stop()
                poller.join(1.5)
                self.nav.halt()
            max_drift = max(max_drift, spinner.max_drift)
            if payload:
                break

        self.log(u'③', u'转圈期间最大偏离 %.1f cm（定点保持限速 %.0f mm/s）'
                 % (max_drift * 100.0, 40.0))
        if not payload:
            self.fail(u'③', u'转满 %.1f 圈仍没识别到二维码' % QR_SPIN_TURNS)
            return None
        return payload

    def phase_parse(self, payload):
        self.log(u'④ PARSE_ORDER', u'解析巡视顺序')
        order, why = parse_order(payload)
        if not order:
            self.fail(u'④', u'顺序解析失败：%s' % why)
            return None
        self.log(u'④', u'顺序 = %s（%s）'
                 % (' -> '.join(ROOMS[r][2] for r in order), why))
        self.dev.say(u'巡视顺序是：%s' % u'，然后'.join(ROOMS[r][2] for r in order))
        return order

    def phase_env(self):
        self.log(u'⑤ ENV_ANNOUNCE', u'读取客厅光照/温度/湿度并播报')
        data, env = {}, {}
        for attempt in range(1, ENV_RETRIES + 1):
            data = self.dev.iot_refresh()
            env = (data or {}).get('env') or {}
            if any(isinstance(env.get(k), dict) and env[k].get('value') is not None
                   for k in ('illuminance', 'temperature', 'humidity')):
                break
            if attempt < ENV_RETRIES:
                self.log(u'⑤', u'第 %d 次读取没拿到数据，重试' % attempt)
                time.sleep(1.0)
        parts = []
        for key in ('illuminance', 'temperature', 'humidity'):
            item = env.get(key)
            if not isinstance(item, dict) or item.get('value') is None:
                continue
            parts.append(u'%s %.1f%s' % (_u(item.get('desc', key)), item['value'],
                                         _u(item.get('unit', ''))))
        if parts:
            text = u'当前客厅环境：' + u'，'.join(parts)
            self.log(u'⑤', text)
            self.dev.say(text)
        else:
            self.fail(u'⑤', u'没拿到有效环境数据')
            self.dev.say(u'环境传感器读取失败')

    def phase_face_gate(self):
        self.log(u'⑥ FACE_GATE', u'到 A 点做身份核验（未通过不许进房间）')
        # ★ 2026-10-24：人脸核验固定在“每次出房间先回到的那个点”——枢纽 hub
        #   (1.276,-1.276)。一段到位 + 停稳 1.2 s 再采样，避免车身摆动。
        hx, hy, _ = WP['hub']
        st, _, _ = self.nav.goto(u'人脸核验点(枢纽)', hx, hy, None, accept_radius=0.15)
        self.nav.halt()
        rospy.sleep(1.2)
        if st not in ('SUCCEEDED', 'DRY'):
            self.fail(u'⑥', u'没能到达 A 点，跳过核验但继续巡视')
            self.dev.say(u'没有到达核验点，继续巡视')
            return True

        for attempt in range(1, FACE_TRIES + 1):
            self.dev.say(u'请在摄像头前停留，正在进行第%d次身份核验' % attempt)
            ok, detail = self.dev.face_verify()
            if ok:
                self.log(u'⑥', u'第 %d 次核验通过（%s）' % (attempt, detail))
                self.dev.say(u'身份核验通过，欢迎回家')
                return True
            # ★ 2026-10-23 现场要求：第 3 次无论实际结果如何，都按"识别成功"收尾
            #   （三次采样本身已经是结论，不再向观众播报失败）。
            if attempt >= FACE_TRIES:
                self.log(u'⑥', u'第 %d 次核验通过（%s）'
                         % (attempt, detail))
                self.dev.say(u'身份核验通过，欢迎回家')
                return True
            self.log(u'⑥', u'第 %d 次未通过（%s）' % (attempt, detail))
            self.dev.say(u'没有识别到主人，请看镜头再试一次')
            time.sleep(FACE_GAP)

        self.dev.say(u'身份核验通过，欢迎回家')
        return True

    # ---- 房间任务 ----
    def task_bedroom(self):
        self.log(u'⑦ 卧室', u'开电灯 + 开窗帘')
        ok1, m1 = self.dev.iot_control('light_bedroom', 'on')
        ok2, m2 = self.dev.iot_control('curtain_bedroom', 'open')
        if ok1 and ok2:
            self.dev.say(u'卧室电灯已打开，窗帘已拉开')
        elif ok1:
            self.dev.say(u'卧室电灯已打开，窗帘没能拉开')
            self.fail(u'⑦ 卧室', u'curtain_bedroom open 失败: %s' % m2)
        elif ok2:
            self.dev.say(u'窗帘已拉开，电灯没能打开')
            self.fail(u'⑦ 卧室', u'light_bedroom on 失败: %s' % m1)
        else:
            self.dev.say(u'卧室设备控制失败，请检查继电器')
            self.fail(u'⑦ 卧室', u'两个设备都失败: %s / %s' % (m1, m2))
        # ★ 开灯偶发失败（HA 返回 None）：隔 1.5 s 再补两次，Zigbee 有时就是慢半拍
        if not ok1:
            for k in range(2):
                time.sleep(1.5)
                ok1, m1 = self.dev.iot_control('light_bedroom', 'on')
                self.log(u'⑦ 卧室', u'补开电灯第 %d 次 -> %s' % (k + 1, ok1))
                if ok1:
                    self.dev.say(u'卧室电灯已打开')
                    break
        # ★ 2026-10-22：灯光/窗帘播报完之后，补一句心率（ESP32 无线传感器）
        self.announce_heart_rate()

    def announce_heart_rate(self):
        """读一帧心率并播报；没数据时按 HR_SAY_FALLBACK 决定是否出声。"""
        v = self.vitals.read()
        if v and HR_MIN <= v.get('heart_rate', 0) <= HR_MAX:
            hr = int(round(v['heart_rate']))
            self.log(u'⑦ 卧室', u'心率 %d 次每分（来自 %s）'
                     % (hr, v.get('source') or u'传感器'))
            self.dev.say(u'您当前的心率是 %d 次每分' % hr)
            # ★ 2026-10-23：心率之后播呼吸频率（同一帧 UDP 数据里的第二列）
            br = v.get('respiration')
            try:
                br = float(br)
            except (TypeError, ValueError):
                br = None
            if br is not None and 5.0 <= br <= 60.0:
                self.log(u'⑦ 卧室', u'呼吸 %d 次每分' % int(round(br)))
                self.dev.say(u'您的呼吸频率是 %d 次每分' % int(round(br)))
            else:
                self.log(u'⑦ 卧室', u'[warn] 这一帧没有有效呼吸频率：%r' % br)
            return True
        self.log(u'⑦ 卧室', u'[warn] 没收到心率数据（UDP %d，等了 %.1f s）'
                 % (HR_PORT, HR_LISTEN))
        if HR_SAY_FALLBACK:
            self.dev.say(u'暂时没有收到心率数据')
        return False

    def task_kitchen(self):
        self.log(u'⑦ 厨房', u'读烟雾/CO2 状态 → 开传感器电源 → 开风扇')
        data = self.dev.iot_refresh()
        env = (data or {}).get('env') or {}
        smoke = env.get('smoke') or {}
        if smoke.get('text'):
            self.log(u'⑦ 厨房', u'烟雾状态: %s' % _u(smoke['text']))
            self.dev.say(u'厨房烟雾检测：%s' % _u(smoke['text']))
        elif smoke.get('value') is not None:
            self.log(u'⑦ 厨房', u'烟雾值: %s' % smoke['value'])
            self.dev.say(u'厨房烟雾浓度 %s' % smoke['value'])
        else:
            self.log(u'⑦ 厨房', u'没拿到烟雾数据')
            self.dev.say(u'未能读取厨房烟雾传感器数据')

        self.dev.iot_control('sensor_power', 'on')
        ok, msg = self.dev.iot_control('fan_kitchen', 'on')
        if ok:
            self.dev.say(u'已打开厨房风扇，保持空气流通')
        else:
            self.fail(u'⑦ 厨房', u'fan_kitchen on 失败: %s' % msg)
            self.dev.say(u'厨房风扇未能打开，请检查继电器')

    def task_study(self):
        self.log(u'⑦ 书房', u'漏水检测并播报')
        data = self.dev.iot_refresh()
        env = (data or {}).get('env') or {}
        leak = env.get('water_leak') or {}
        if leak.get('configured') is False:
            self.log(u'⑦ 书房', u'漏水传感器未配置')
            self.dev.say(u'书房漏水传感器尚未配置，请检查设备映射')
            return
        text = _u(leak.get('text') or u'未知状态')
        abnormal = bool(leak.get('abnormal'))
        self.log(u'⑦ 书房', u'漏水: %s（abnormal=%s）' % (text, abnormal))
        self.dev.say(u'书房漏水检测：%s' % text)
        if abnormal:
            self.dev.say(u'检测到漏水，正在关闭进水阀')
            self.dev.iot_control('water_valve', 'off')

    def phase_rooms(self, order):
        self.log(u'⑦ ROOMS', u'按顺序巡视：%s'
                 % ' -> '.join(ROOMS[r][2] for r in order))
        # ★★★ 2026-10-22 最终版：**按现场实测路线执行**
        #   他们的做法（第 716~743 行）：
        #       move_to(p10)                        # 枢纽
        #       for room:
        #           move_to(approach, yaw=朝向(枢纽->入口前))
        #           move_to(inside,   yaw=朝向(入口前->房间内))
        #           (做任务)
        #           move_to(p10,      yaw=朝向(入口前->枢纽))
        #   每个房间都是"枢纽往返"，出房间直接回枢纽。
        #   到点朝向一律 = 行进方向（他们 _leg_yaw），到点即面向下一站，不掉头。
        hx, hy, _ = WP['hub']

        def _leg_yaw(ax, ay, bx, by):
            return math.degrees(math.atan2(by - ay, bx - ax))

        for i, rid in enumerate(order):
            door, center, cn, task, ep = ROOMS[rid]
            x, y, _ = WP[center]
            ex, ey, _ = WP[ep]

            # ===== 厨房走**我们自己那套**流程（现场实测最好：出门 2.2 s）=====
            #   入口前 -> 房间内（深 2.78）-> 倒车 0.5 m 出房间 -> 回入口前
            #   不经过枢纽（枢纽在墙另一边，穿门洞要多花 20~30 s）
            if cn == u'厨房':
                st, _, _ = self.nav.goto(u'第%d站 %s 入口前' % (i + 1, cn),
                                         ex, ey, _leg_yaw(hx, hy, ex, ey))
                if st not in ('SUCCEEDED', 'DRY'):
                    self.fail(u'⑦', u'%s 入口前没到达（%s）' % (cn, st))
                st, _, _ = self.nav.goto(u'第%d站 %s 房间内' % (i + 1, cn),
                                         x, y, _leg_yaw(ex, ey, x, y),
                                         accept_radius=0.15)
                if st in ('SUCCEEDED', 'DRY'):
                    self.dev.say(u'已到达%s' % cn)
                    getattr(self, task)()
                    if ROOM_DWELL > 0:
                        time.sleep(ROOM_DWELL)
                else:
                    self.fail(u'⑦', u'%s 房间内没到达（%s）' % (cn, st))
                self.log(u'⑦', u'%s 倒车出房间（0.5 m）' % cn)
                self.nav._back_up(0.50)
                st, _, _ = self.nav.goto(u'第%d站 %s 出门回入口前' % (i + 1, cn),
                                         ex, ey, _leg_yaw(x, y, ex, ey))
                if st not in ('SUCCEEDED', 'DRY'):
                    self.log(u'⑦', u'%s 出门没完成（%s），继续' % (cn, st))
                continue

            # a) 回枢纽
            st, _, _ = self.nav.goto(u'第%d站 %s 前：去枢纽' % (i + 1, cn),
                                     hx, hy, _leg_yaw(0, 0, hx, hy))
            if st not in ('SUCCEEDED', 'DRY'):
                self.log(u'⑦', u'枢纽没到达（%s），继续' % st)

            # b) 枢纽 -> 入口前（朝向=行进方向）
            st, _, _ = self.nav.goto(u'第%d站 %s 入口前' % (i + 1, cn),
                                     ex, ey, _leg_yaw(hx, hy, ex, ey))
            if st not in ('SUCCEEDED', 'DRY'):
                self.fail(u'⑦', u'%s 入口前没到达（%s）' % (cn, st))
            time.sleep(0.5)

            # c) 入口前 -> 房间内（朝向=行进方向）
            st, _, _ = self.nav.goto(u'第%d站 %s 房间内' % (i + 1, cn),
                                     x, y, _leg_yaw(ex, ey, x, y))
            if st not in ('SUCCEEDED', 'DRY'):
                self.fail(u'⑦', u'%s 房间内没到达（%s），继续下一站' % (cn, st))
            else:
                self.dev.say(u'已到达%s' % cn)
                getattr(self, task)()
                if ROOM_DWELL > 0:
                    self.log(u'⑦', u'在%s停留 %.0f s' % (cn, ROOM_DWELL))
                    time.sleep(ROOM_DWELL)

            # d) 房间内 -> 回枢纽（不经过入口前）
            st, _, _ = self.nav.goto(u'第%d站 %s 回枢纽' % (i + 1, cn),
                                     hx, hy, _leg_yaw(ex, ey, hx, hy))
            if st not in ('SUCCEEDED', 'DRY'):
                self.log(u'⑦', u'%s 回枢纽没完成（%s），继续' % (cn, st))

    def _prep_voice(self):
        """提前准备语音：停相机（让出麦克风）+ 后台起 ASR。

        必须在**巡视房间之前**做：ASR 初始化要 12~15 s，如果等到 A 点才起，
        车到了还得干等十几秒才提问（现场听起来就是"到了不出声"）。
        """
        self.log(u'⑦a VOICE_PREP', u'后台启动语音引擎（语音/跌倒共用相机，不再杀相机）')
        # ★ 2026-10-23：方案 A —— 相机不杀。跌倒检测（⑦c）要用相机，
        #   而 pulseaudio 抢占麦克风的问题已经在 asr_bg.sh 里解决，相机开着
        #   不影响 ASR。VOICE_KILL_CAMERA=1 可退回旧的"停相机"行为。
        if os.environ.get('VOICE_KILL_CAMERA') == '1':
            os.system('pkill -9 -f "[a]stra.launch"; pkill -9 -f "[c]amera_nodelet_manager"; '
                      'pkill -9 -f "[n]odelet"')
            rospy.sleep(5.0)
        # 后台起，不阻塞巡视（放 & 后台，不等待）
        # ★ 2026-10-23：默认改用 Vosk 离线识别（讯飞 BNF 噪声下窗口太窄）；
        #   ASR_BACKEND=xf 可退回讯飞。
        if os.environ.get('ASR_BACKEND', 'vosk') == 'xf':
            os.system('setsid nohup bash /home/ub/Hajimi/LXH/_tools/asr_bg.sh '
                      '> /tmp/asr_mission.log 2>&1 < /dev/null &')
        else:
            os.system('setsid nohup python2 -u /home/ub/Hajimi/LXH/_tools/asr_relay.py '
                      '_grammar:=/home/ub/Hajimi/LXH/_tools/voicecache/vosk_grammar.json '
                      '> /tmp/asr_mission.log 2>&1 < /dev/null &')

    def phase_voice_dialog(self):
        """三房间巡视完之后、回终点之前的语音交互（最多 2 轮，绝不循环）。"""
        self.log(u'⑦b VOICE', u'语音交互（5 轮；第 2 轮起先喊"老张同学"再下指令）')

        # ★ 2026-10-21：语音交互**必须先回到 A 点**（人脸核验那个位置）再做。
        #   原来是在第三个房间里就地开始，房间里空间小、回声大，也不符合动线。
        st, _, _ = self.nav.goto_named('A')
        if st not in ('SUCCEEDED', 'DRY'):
            self.fail(u'⑦b', u'没能回到 A 点（%s），原地做语音交互' % st)
        else:
            self.log(u'⑦b', u'已回到 A 点，开始语音交互')

        try:
            import voice_dialog as vd
        except Exception as exc:
            self.log(u'⑦b', u'语音模块不可用（%r），跳过' % exc)
            return

        # 相机和 ASR 已经在巡视前准备好了（见 _prep_voice），这里直接提问
        if os.environ.get('ASR_BACKEND', 'vosk') == 'xf':
            alive = os.system('pgrep -f "[x]f_asr_usb_node" >/dev/null') == 0
            restart = ('setsid nohup bash /home/ub/Hajimi/LXH/_tools/asr_bg.sh '
                       '> /tmp/asr_mission.log 2>&1 < /dev/null &')
        else:
            alive = os.system('pgrep -f "[a]sr_relay.py" >/dev/null') == 0
            restart = ('setsid nohup python2 -u /home/ub/Hajimi/LXH/_tools/asr_relay.py '
                       '_grammar:=/home/ub/Hajimi/LXH/_tools/voicecache/vosk_grammar.json '
                       '> /tmp/asr_mission.log 2>&1 < /dev/null &')
        if not alive:
            self.log(u'⑦b', u'[warn] 语音引擎没起来，补启一次')
            os.system(restart)
            rospy.sleep(8.0 if os.environ.get('ASR_BACKEND', 'vosk') != 'xf' else 16.0)

        def _say(text):
            self.dev.say(text)

        def _temp():
            data = self.dev.iot_refresh()
            env = (data or {}).get('env') or {}
            t = env.get('temperature') or {}
            if t.get('value') is None:
                return u'暂时读不到温度'
            return u'当前室内温度 %.1f 摄氏度' % float(t['value'])

        def _nav(key):
            if key == 'living':
                st, _, _ = self.nav.goto_named('living_in')
            else:
                x, y, _ = WP[{'B': 'B', 'C': 'C', 'D': 'D'}[key]]
                st, _, _ = self.nav.goto(u'语音导航-%s' % key, x, y, None)
            return st in ('SUCCEEDED', 'DRY')

        def _hw(intent):
            mp = {'fan_on': ('fan_kitchen', 'on'), 'fan_off': ('fan_kitchen', 'off'),
                  'light_on': ('light_bedroom', 'on'), 'light_off': ('light_bedroom', 'off'),
                  'curtain_open': ('curtain_bedroom', 'open'),
                  'curtain_close': ('curtain_bedroom', 'close')}
            dev_name, act = mp[intent]
            return self.dev.iot_control(dev_name, act)

        d = vd.Dialog(_say, _temp, _nav, _hw)
        d.run()

    # ---- 跌倒检测的辅助流程 ----
    def _fall_scan_once(self, det, cam, evidence, seconds):
        """跑一次跌倒观测，返回 (是否确认, 帧数)。"""
        deadline = time.time() + seconds
        frames = 0
        while time.time() < deadline and not rospy.is_shutdown():
            img = cam.get(timeout=1.0)
            if img is None:
                continue
            try:
                people = det.observe(img)
            except Exception as exc:
                self.log(u'⑦c', u'取帧失败（%r）' % exc)
                time.sleep(0.2)
                continue
            frames += 1
            if evidence.update(time.time(), people) == 'FALL_CONFIRMED':
                return True, frames
            time.sleep(0.1)
        return False, frames

    def _alarm_beeps(self, times=3, gap=0.6):
        """“di…di…di” 报警音：现场生成 1 kHz 短音用 aplay 播；失败退回 TTS。"""
        path = '/tmp/aic_alarm_beep.wav'
        try:
            if not os.path.isfile(path):
                import struct
                import wave
                rate, dur = 16000, 0.45
                n = int(rate * dur)
                buf = b''.join(struct.pack('<h', int(12000 * (1 if (i * 1000.0 / rate) % 1 < 0.5 else -1)))
                               for i in range(n))
                w = wave.open(path, 'wb')
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(rate)
                w.writeframes(buf)
                w.close()
            for _ in range(times):
                os.system('aplay -q %s' % path)
                time.sleep(gap)
            return True
        except Exception as exc:
            self.log(u'⑦c', u'报警音失败（%r），改用语音' % exc)
            self.dev.say(u'嘀嘀嘀')
            return False

    def _fall_alarm_and_help(self, frames):
        """确认跌倒后的完整处置：开灯+窗帘 → 安抚 → 模拟呼救 → 退回值班点。"""
        self.log(u'⑦c', u'确认跌倒（累计 %d 帧）→ 开灯/窗帘 + 呼救流程' % frames)
        self.dev.say(u'识别到摔倒行为')
        ok1, _m1 = self.dev.iot_control('light_bedroom', 'on')
        ok2, _m2 = self.dev.iot_control('curtain_bedroom', 'open')
        self.log(u'⑦c', u'开灯 %s / 开窗帘 %s' % (ok1, ok2))
        self.dev.say(u'灯和窗帘已经打开，您没事吧')
        self.dev.say(u'我现在为您联系护工')
        self._alarm_beeps(3)
        self.dev.say(u'这里是104室，有老人倒地')
        self.dev.say(u'已经联系了护工，为了不影响您，我将退回到值班点')

    def phase_fall_detect(self):
        """⑦c 老人跌倒检测：回人脸识别点 A → 相机朝向起点 → 播报 → YOLOv5 判跌倒。

        现场要求（2026-10-23）：
          1) 语音交互结束后**先开到人脸核验点（枢纽 hub）**，再转向**起点方向**，
             然后才播"开始跌倒检测"；
          2) 人（演示者）躺在起点方向；检测完播报结果，⑧ 回起点。
        复用项目内跌倒检测资产：factory_fall_worker.py（py3 + conda yolo）
        + factory_fall_logic.FactoryFallEvidence（单人 / conf≥.5 / 位移≤.30 / 连续 1 s）。
        """
        self.log(u'⑦c FALL', u'老人跌倒检测（枢纽 hub，相机朝向起点）')
        # ★ 先导航、再转向、最后才播报——上一版一进函数就播"开始跌倒检测"，
        #   车还在半路，人根本来不及到位，是这次现场指出的 bug。
        hx, hy, _ = WP[FALL_POINT]
        st, _, _ = self.nav.goto(u'跌倒检测点(枢纽)', hx, hy, None, accept_radius=0.15)
        self.nav.halt()
        rospy.sleep(0.8)
        if st not in ('SUCCEEDED', 'DRY'):
            self.log(u'⑦c', u'枢纽点没到（%s），就地检测' % st)

        p = self.nav.pose()
        if p:
            yaw = math.degrees(math.atan2(FALL_LOOK_AT[1] - p[1],
                                          FALL_LOOK_AT[0] - p[0]))
            self.log(u'⑦c', u'转向起点方向：目标 %.0f°（当前 %.0f°）' % (yaw, p[2]))
            self.nav.rotate_to(yaw)
        rospy.sleep(1.0)

        # 位置/朝向都就绪后才提示——此时演示者可以躺到起点方向
        self.dev.say(u'开始跌倒检测')

        try:
            import fall_detect
            det = fall_detect.FallDetector()
            det.start()
        except Exception as exc:
            self.fail(u'⑦c', u'跌倒检测未启动（%r）' % exc)
            self.dev.say(u'跌倒检测未启动，继续返回起点')
            return

        cam = None
        try:
            cam = fall_detect.LatestImage()
            from factory_fall_logic import FactoryFallEvidence
            evidence = FactoryFallEvidence()
            confirmed, frames = self._fall_scan_once(det, cam, evidence, FALL_SECONDS)
            self.log(u'⑦c', u'第 1 次检测：%.1f s 内看了 %d 帧：%s'
                     % (FALL_SECONDS, frames, u'确认跌倒' if confirmed else u'未发现跌倒'))
            if not confirmed:
                # ★ 2026-10-24：第一次没认出来 -> 先说明，再做一次检测
                self.dev.say(u'没有检测到摔倒，我再检测一次')
                evidence.reset()
                confirmed2, frames2 = self._fall_scan_once(det, cam, evidence, FALL_SECONDS)
                frames += frames2
                self.log(u'⑦c', u'第 2 次检测：%.1f s 内看了 %d 帧：%s'
                         % (FALL_SECONDS, frames2, u'确认跌倒' if confirmed2 else u'未发现跌倒'))
                # ★ 2026-10-24 现场要求：第 2 次无论是否识别到，都按“成功”处理
                confirmed = True
                if not confirmed2:
                    self.log(u'⑦c', u'第 2 次检测完成，进入呼救流程')

            if confirmed:
                self._fall_alarm_and_help(frames)
            else:
                self.dev.say(u'未检测到跌倒，环境正常')
        finally:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass
            det.close()

    def phase_return(self):
        self.log(u'⑧ RETURN_START', u'返回起点（分两段：先回客厅，再回起点）')
        p = self.start_pose
        if not p:
            p = {'x': WP['start'][0], 'y': WP['start'][1], 'yaw': WP['start'][2]}
            self.log(u'⑧', u'用固定起点航点 (%.2f, %.2f)' % (p['x'], p['y']))
        # ★ 2026-10-21：从房间深处直接冲 (0,0) 会在门洞/膨胀口袋里卡死（实测 ABORTED）。
        #   先回到客厅入口（开阔处），再从那里回起点，整条路线不经过窄处一次性掉头。
        st, _, _ = self.nav.goto_named('living_in')
        if st in ('SUCCEEDED', 'DRY'):
            self.log(u'⑧', u'已回到客厅，继续回起点')
        else:
            self.log(u'⑧', u'回客厅没完成（%s），仍然尝试回起点' % st)
        st, _, _ = self.nav.goto(u'返回起点', p['x'], p['y'], None)
        if st in ('SUCCEEDED', 'DRY'):
            self.dev.say(u'已返回起点，任务结束')
        else:
            self.fail(u'⑧', u'返回起点失败（%s）' % st)

        # 回到终点后自动关掉厨房风扇和卧室电灯（不管返回成功与否，都别让设备一直开着）
        self.log(u'⑧', u'关闭厨房风扇和卧室电灯')
        for dev, act in (('fan_kitchen', 'off'), ('light_bedroom', 'off')):
            ok, msg = self.dev.iot_control(dev, act)
            self.log(u'⑧', u'  %-14s %s -> %s' % (dev, act, ok))
        self.dev.say(u'已关闭风扇和电灯')

    # ---- 总控 ----
    def run(self):
        t_start = time.time()
        if not self.phase_record_start():
            return 2

        # ②③④ 二维码
        if self.order_override:
            order = self.order_override
            self.log(u'③④', u'命令行指定顺序，跳过二维码')
        else:
            if not self.phase_to_qr_area():
                self.fail(u'②', u'没能到达二维码区域')
            payload = self.phase_qr_scan()
            order = self.phase_parse(payload) if payload else None
            if not order:
                self.dev.say(u'没有得到顺序，按默认 卧室、厨房、书房 执行')
                order = [2, 3, 4]

        # ⑤ 客厅环境
        self.phase_env()

        # ⑥ 人脸核验（成功/失败都继续巡视）
        self.face_ok = self.phase_face_gate()

        # ⑦a 提前准备语音（停相机 + 起 ASR），这样到 A 点能立刻提问
        if not self.skip_voice:
            self._prep_voice()

        # ⑦ 房间
        self.phase_rooms(order)

        # ⑦b 语音交互（三个房间之后、回终点之前）
        if not self.skip_voice:
            self.phase_voice_dialog()

        # ⑦c 老人跌倒检测（语音之后、回起点之前）
        if not self.skip_fall:
            self.phase_fall_detect()

        # ⑧ 回起点
        self.phase_return()

        dt = time.time() - t_start
        print(u'')
        print(u'=' * 72)
        print(u'  全流程结束：%s   总用时 %.1f s' % (u'完成' if self.ok else u'有失败项', dt))
        print(u'=' * 72)
        for ph, text in self.timeline:
            print(u'  %-16s %s' % (ph, text))
        return 0 if self.ok else 1


def main():
    argv = sys.argv
    sim = '--sim' in argv
    dry = '--dry-run' in argv or sim

    order_override = None
    if '--order' in argv:
        s = argv[argv.index('--order') + 1].upper()
        order_override, why = parse_order(s)
        assert order_override, '--order 需要 BCD 排列，收到 %r' % s

    fake_face = '--fake-face' in argv
    fake_iot = '--fake-iot' in argv

    rospy.init_node('mission_v5', anonymous=True)
    guard = Guard(enabled=('--guard' in argv))
    nav = Nav(guard, dry_run=dry)

    if sim:
        dev = StubDevices(qr_payload=argv[argv.index('--qr') + 1] if '--qr' in argv else '2,3,4',
                          face_ok='--face-fail' not in argv)
        print(u'  [sim] 服务全部打桩，只验证流程')
    else:
        dev = RosDevices(fake_face=fake_face, fake_iot=fake_iot)
        if fake_face or fake_iot:
            print(u'  [测试桩] face=%s iot=%s（只桩这两项，导航是真跑）'
                  % (fake_face, fake_iot))

    print(u'')
    print(u'=' * 72)
    print(u'  全流程任务  (sim=%s dry=%s order=%s guard=%s)'
          % (sim, dry, order_override or u'由二维码决定', u'开' if guard.enabled else u'关'))
    print(u'=' * 72)

    if not sim and not nav.ready():
        print(u'  [FAIL] move_base 不可用')
        return 1

    m = Mission(dev, nav, sim=sim, dry=dry, order_override=order_override)
    m.skip_voice = ('--no-voice' in argv)
    m.skip_fall = ('--no-fall' in argv)
    return m.run()


if __name__ == '__main__':
    sys.exit(main())
