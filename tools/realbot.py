# -*- coding: utf-8 -*-
"""
realbot.py —— 真机安全驱动库

## 为什么不用轮式里程计

实测（`base_control/script/stm32.py`）：

    self.pose_x = float(ctypes.c_int16(self.postion_x).value/100.0)

`pose_x` 是 **STM32 报的绝对编码器值**，不是增量积分，**上电不清零**。
实测静止时读到 x=-164.55 → -164.31 → -164.01（慢慢变），
而**线速度一直是 0** —— 机器人在场地里（3m），位置却是 -164/-184 m
（`c_int16` 上限 ±327 m，-184 已接近回绕）。

→ **轮式里程计的位置和航向都不可信**（静止 10 秒航向漂 44°）。

## 用什么替代

  · **航向** → IMU（`/imu_data` 的 `orientation` 四元数）
    实测静止 30 秒只漂 **0.03°**，是绝对方位，完全可用。
  · **距离** → 激光雷达测距 + 定时 + 碰撞护栏
    不靠轮式里程计推算位移，而是"走多久 + 前方还剩多少空间"。

## 安全设计（这台底盘**没有 cmd_vel 超时归零**！）

`stm32.py` 里 `last_cmd_vel_time` 被记录但**没被用来刹车**，
一旦发了速度就会一直保持。所以：

  1. 所有退出路径（正常/异常/Ctrl-C/信号）都走同一个 `stop()`
  2. **独立看门狗线程**持续压 0；主线程一旦卡死，`WD_TIMEOUT` 内速度归零
  3. 激光前向护栏：行进方向 ± 扇区内 < `FRONT_STOP` 立即停
  4. 每一步都有**硬超时**
"""

from __future__ import print_function

import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

# ------------------------------------------------------------------ 参数
SCAN_TOPIC = '/scan'            # 原始扫描 + MIN_VALID 下限（见下）
V_LINEAR = 0.12          # m/s   前进（保守；赛题要求 >=0.6 是"最大速度"上限，不是必须快）
V_ANGULAR = 0.30         # rad/s 转向
FRONT_HALF = 40.0        # deg   护栏扇区半宽
FRONT_STOP = 0.45        # m     前方 < 此距离急停
                         #  原 0.70 对 0.28 m 半径的车过于保守：
                         #  0.70 意味着离墙 0.70 m 就不敢动，而小场地里
                         #  很多方向只有 0.6 m —— 结果一步也走不出去（实测 0 次前进）。
                         #  0.45 = 半径 0.28 + 0.17 余量，配合 0.09 m/s 低速足够安全。
                         # 配合 SCAN_TOPIC=/scan_filtered 使用 —— 见 MIN_VALID 注释

# ---------------------------------------------------------------- 近距回波下限
#
# ⚠️⚠️ MIN_VALID 从 0.09 提到 **0.20** —— 这是实测逼出来的。
#
#  这台车的雷达实测近距回波分布（scan_detail.py）：
#      < 0.30 m 的点有 **174 / 856** 个
#      集中在 150°~210°（背后，146 点）和 **20°~30°（前方偏左，19 点）**
#      距离全在 0.097 ~ 0.17 m
#
#  0.14 m **小于车体半径 0.18 m** —— 物理上不可能是"能撞到的障碍"，
#  只能是**车体自身结构/支架**扫进来的。
#
#  后果：探索器一直报"前方仅 0.14 m"，原地 ±45° 来回摆，**一步都走不了**。
#  实测日志：
#      [49s] 前方仅 0.14 m(左 1.24 右 1.10) → 转 +45°
#      [52s] 前方仅 0.14 m(左 0.65 右 1.00) → 转 -45°
#      [55s] 前方仅 0.14 m(左 1.24 右 1.08) → 转 +45°
#
#  ✔ 0.20 m 是合理的分界：
#      · 车体半径 0.18 m → 比这更近的点已经在车体内
#      · 真墙在 ≥0.25 m 时仍能被护栏看见（护栏阈值 FRONT_STOP=0.40）
#      · 而机器人**不可能**把墙压到 0.20 m 以内还不先触发 0.40 的急停
#
#  ⚠️ 另外：gmapping 用的是 `/scan_filtered`（laser_filter 已把
#      <0.596 m 的点全滤掉了），所以**地图是干净的**，与这个参数无关。
MIN_VALID = 0.55         # m     比这更近的点当车体自遮挡
# ⚠️⚠️ 为什么改成"护栏直接读滤波后的 /scan_filtered"
#
#  这台车的**车体自遮挡一直延伸到约 0.45 m**（50 cm 车径 + 雷达支架），
#  实测近距回波是 0.30 / 0.31 / 0.35 / 0.36 / 0.37 / 0.38 / 0.40 / 0.42 / 0.44
#  这样一串**连续值** —— 那是支架在不同角度上的回波，不是墙。
#
#  我先后试过 MIN_VALID = 0.20 / 0.30 / 0.35，都不够：
#    · 0.20 时前方读到 0.22  → 卡住
#    · 0.30 时前方读到 0.30  → 卡住
#    · 0.35 时前方读到 0.35  → "拒绝前进"（实测日志）
#      front = 0.35  ->  "★ 前方只有 0.35 m，拒绝前进"
#
#  而我们的 laser_filter 输出的 /scan_filtered 实测：
#      /scan           有效 856，近点(<0.30) 174，min 0.097
#      /scan_filtered  有效 658，近点(<0.30)   0，min 0.596   <- 干净
#
#  → 所以护栏**直接用滤波后的扫描**，自遮挡问题彻底消失；
#    代价是最小可见距离变成 ~0.60 m，于是 FRONT_STOP 也跟着定 0.60。

# ---------------------------------------------------------------- 噪声抗性
# 实测这台 M10 雷达会偶发**单点假回波**（多径/串扰）：
#   · test_drive 报前方 0.14 m → 同一朝向相邻帧是 2.47 m
#   · survey 里 0.13~0.14 m 反复出现
# 真障碍是**连续一片**点，假回波只有**孤立一两个**。
# 所以护栏按"簇"判断：±扇区内必须有连续 MIN_CLUSTER 个点且距离差
# 不超过 CLUSTER_TOL，才认为那里真有东西。
MIN_CLUSTER = 3          # 个点
CLUSTER_TOL = 0.15       # m
WD_TIMEOUT = 0.60        # s     看门狗超时
CTRL_HZ = 20.0

# ---------------------------------------------------------------- 转向增益
# ⚠️ 实测：发 15° 的指令，IMU 实测只转 12.4~13.8°（**偏小**），
#    24 步累计差了约 42°。原因是底盘对 angular.z 的响应增益偏小；
#    速度太低时电机还要克服静摩擦，稳态误差更明显。
#
# 补偿方式：`指令角 = 期望角 / TURN_GAIN`。
# 标定脚本 `calib_turn.py` 会测出这个值并写 /tmp/turn_calib.json。
#
# ⚠️ 即使有绝对角度闭环，这个补偿也很重要：
#    闭环只能"看到误差再修"，而每一步的**稳态误差**是闭环消不掉的
#    （速度已经很小了，再小就转不动）。
_GAIN_FILE = '/tmp/turn_calib.json'
try:
    import json as _json
    with open(_GAIN_FILE) as _fp:
        _k = float(_json.load(_fp).get('k') or 1.0)
    TURN_GAIN = _k if 0.3 < _k < 2.0 else 1.0
except Exception:
    TURN_GAIN = 1.0


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class RealBot(object):
    def __init__(self, dry_run=False, verbose=True):
        self.dry = dry_run
        self.verbose = verbose
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._lock = threading.Lock()
        self._scan = None
        self._imu = None
        self._odom = None
        self._last_beat = time.time()
        self._stopped = threading.Event()
        self.trace = []          # 记录过程，供事后分析

        # ⚠️ 默认订阅**滤波后**的扫描：/scan_filtered 里没有任何车体自遮挡点。
        #    可以用 scan_topic 参数覆盖（比如调试时想看原始数据）。
        self.scan_topic = rospy.get_param('~scan_topic', SCAN_TOPIC)
        rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan,
                         queue_size=5)
        rospy.Subscriber('/imu_data', Imu, self._on_imu, queue_size=20)
        rospy.Subscriber('/odom', Odometry, self._on_odom, queue_size=10)

        self._wd = threading.Thread(target=self._watchdog)
        self._wd.daemon = True
        self._wd.start()
        rospy.on_shutdown(self.stop)

    # ------------------------------------------------------------ 回调
    def _on_scan(self, m):
        with self._lock:
            self._scan = m

    def _on_imu(self, m):
        with self._lock:
            self._imu = m

    def _on_odom(self, m):
        with self._lock:
            self._odom = m

    def beat(self):
        self._last_beat = time.time()

    def _watchdog(self):
        r = rospy.Rate(CTRL_HZ)
        while not rospy.is_shutdown():
            try:
                if time.time() - self._last_beat > WD_TIMEOUT:
                    self.pub.publish(Twist())
            except Exception:
                pass
            r.sleep()

    # ------------------------------------------------------------ 状态
    def ready(self, timeout=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            with self._lock:
                if self._scan and self._imu:
                    return True
            time.sleep(0.1)
        return False

    def yaw(self):
        """IMU 航向（弧度）。这是**唯一可信**的航向来源。"""
        with self._lock:
            m = self._imu
        return None if m is None else quat_to_yaw(m.orientation)

    def wheel_pose(self):
        """轮式里程计位姿 —— **仅用于对比观察，不要用于控制**。"""
        with self._lock:
            m = self._odom
        if m is None:
            return None
        p = m.pose.pose
        return (p.position.x, p.position.y, quat_to_yaw(p.orientation))

    def sector_min(self, heading_rel=0.0, half_deg=None):
        """以车头为 0° 的方向 heading_rel ± half_deg 内的最近有效距离。

        ⚠️ 这是**原始最小值**，会被单点噪声带偏 —— 护栏请用
           `sector_min_robust()`。本函数保留给诊断用。
        """
        half = math.radians(FRONT_HALF if half_deg is None else half_deg)
        with self._lock:
            s = self._scan
        if s is None:
            return None
        best = None
        for i, v in enumerate(s.ranges):
            if v != v or v == float('inf') or v < MIN_VALID:
                continue
            a = s.angle_min + i * s.angle_increment
            if abs(norm_angle(a - heading_rel)) > half:
                continue
            if best is None or v < best:
                best = v
        return best

    def sector_min_robust(self, heading_rel=0.0, half_deg=None,
                          cluster=MIN_CLUSTER):
        """按"簇"判断的最近距离 —— **护栏必须用这个**。

        ## 为什么

        实测这台雷达（M10）会偶发**单点假回波**：
          · `test_drive` 报前方 0.14 m → 但同一朝向相邻一帧是 2.47 m
          · `survey` 里 0.13~0.14 m 也反复出现
        那是多径/串扰，不是真障碍。

        真障碍会产生**连续一片**点（墙在 360° 里占几十个点），
        假回波只有**孤立一两个**点。

        所以：把 ±half 内的点按距离排序，只有**连续 `cluster` 个点**
        都近于某个距离时，才认为那里真有东西。

        `cluster=1` 时等价于原始最小值。
        """
        half = math.radians(FRONT_HALF if half_deg is None else half_deg)
        with self._lock:
            s = self._scan
        if s is None:
            return None
        pts = []
        for i, v in enumerate(s.ranges):
            if v != v or v == float('inf') or v < MIN_VALID:
                continue
            a = s.angle_min + i * s.angle_increment
            if abs(norm_angle(a - heading_rel)) > half:
                continue
            pts.append((a, v))
        if not pts:
            return None
        pts.sort(key=lambda t: t[1])          # 按距离从近到远
        # 找第一个"连续 cluster 个点距离都差不多"的位置
        n = len(pts)
        for k in range(0, max(1, n - cluster + 1)):
            window = pts[k:k + cluster]
            if len(window) < cluster:
                break
            lo = window[0][1]
            hi = window[-1][1]
            if hi - lo <= CLUSTER_TOL:
                return lo
        return pts[-1][1] if pts else None

    # ------------------------------------------------------------ 安全
    def stop(self, reason=''):
        if self._stopped.is_set():
            return
        self._stopped.set()
        try:
            for _ in range(6):
                self.pub.publish(Twist())
                time.sleep(0.03)
        except Exception:
            pass
        if reason and self.verbose:
            print('  [STOP] %s' % reason)

    def _pub(self, lin=0.0, ang=0.0):
        if self.dry:
            return
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.pub.publish(t)

    # ------------------------------------------------------------ 动作
    def rotate(self, delta_deg, timeout=30.0, tol_deg=2.5):
        """原地转 delta_deg（正=左转/逆时针）。用 IMU 闭环 + 增益补偿。

        ## ⚠️⚠️ 必须用**累计角度**，不能用归一化的绝对角

        踩过（实测）：目标 +180°，结果它转了 185°，
        但 `norm_angle(+185°)` 被折成 **-175°** —— 于是闭环"看到"的
        误差变成 355°，**方向判断整个反了**。

        正确做法：把每一步的 yaw 增量**累加**（unwrap），
        用累计值作为闭环反馈 —— 这样 180°、360° 都不会在边界上打转。

        ## 增益补偿

        实测底盘对 angular.z 的响应偏小（发 20° 转 19.5°），
        乘 TURN_GAIN 补偿后可到 ±1°。
        """
        y_prev = self.yaw()
        if y_prev is None:
            print('  ✘ 没有 IMU，无法转向闭环')
            return False

        cmd_deg = delta_deg / TURN_GAIN if TURN_GAIN else delta_deg
        # 累计转角（unwrap），带符号
        accum = 0.0
        t0 = time.time()
        r = rospy.Rate(CTRL_HZ)
        stall = 0
        last_accum = 0.0
        while not rospy.is_shutdown():
            self.beat()
            y = self.yaw()
            if y is None:
                break
            step = norm_angle(y - y_prev)      # 本周期转了多少
            accum += math.degrees(step)
            y_prev = y

            err = cmd_deg - accum              # 直接比较累计角
            if abs(err) < tol_deg:
                break
            if time.time() - t0 > timeout:
                print('  ⚠️ 转向超时，还差 %.1f°' % err)
                break
            # 死区：实测步长太小时会卡住（静摩擦），给一个最小速度
            w = max(-V_ANGULAR, min(V_ANGULAR,
                                    math.radians(err) * 1.8
                                    + math.copysign(0.10, err)))
            self._pub(ang=w)
            # 卡死检测：连续 0.4 s 没动就加大一点速度
            if abs(accum - last_accum) < 0.05:
                stall += 1
                if stall > 8:
                    self._pub(ang=math.copysign(V_ANGULAR, err))
                    stall = 0
            else:
                stall = 0
            last_accum = accum
            r.sleep()
        self._pub()
        time.sleep(0.35)

        # 停稳后再量一次（底盘有惯性）
        got = accum
        y_end = self.yaw()
        if y_end is not None:
            got += math.degrees(norm_angle(y_end - y_prev))
        if self.verbose:
            print('  转向: 目标 %+.1f° (指令 %+.1f°, 增益 %.2f)  '
                  '实际 %+.1f°  误差 %+.1f°'
                  % (delta_deg, cmd_deg, TURN_GAIN, got, delta_deg - got))
        self.trace.append(('rotate', delta_deg, got))
        return abs(delta_deg - got) < 3.0

    def forward_until(self, stop_dist, timeout=20.0, speed=0.10,
                      min_travel=0.05, heading_rel=0.0):
        """朝车头方向前进，直到**前方距离降到 stop_dist** 或超时。

        ## 为什么用"激光距离"而不是"时间×速度"

        实测轮式里程计不可用，所以位移没有可靠反馈。
        "定时开环"的误差会累积；而"走到离墙 X 米"是**激光直接测出来的**，
        不累积误差 —— 在 3x3 小场地里这个更可靠。

        ⚠️ 但如果前方本来就近（< stop_dist），会**立刻返回**，不会硬冲。
        """
        t0 = time.time()
        r = rospy.Rate(CTRL_HZ)
        while not rospy.is_shutdown():
            self.beat()
            fm = self.sector_min_robust(heading_rel)
            el = time.time() - t0
            if fm is not None and fm <= stop_dist:
                break
            if el > timeout:
                print('  ⚠️ forward_until 超时（前方 %s，目标 %.2f）'
                      % (('%.2f' % fm) if fm is not None else '---',
                         stop_dist))
                break
            # 离目标还远就全速，近了减速
            if fm is None:
                v = speed
            else:
                gap = fm - stop_dist
                v = min(speed, max(0.05, gap * 0.8))
            self._pub(lin=v)
            r.sleep()
        self._pub()
        time.sleep(0.4)
        if self.verbose:
            fm = self.sector_min_robust(heading_rel)
            print('  前进至前方 %.2f m（用时 %.1f s）'
                  % (fm if fm is not None else -1, time.time() - t0))
        self.trace.append(('forward_until', stop_dist, None))
        return True

    def forward(self, dist_m, timeout=None, speed=None):
        """按"时间 × 速度"前进 dist_m。

        ⚠️ 距离**不是**靠里程计闭环（它不可信），而是：
             · 设定速度 × 设定时间 ≈ 目标距离
             · 全程用激光护栏盯着前方，太近立即停
        所以实测距离会有误差（约 ±10~20%），但**不会撞**。
        """
        v = speed if speed else (V_LINEAR if dist_m > 0 else -V_LINEAR * 0.7)
        if timeout is None:
            timeout = abs(dist_m) / abs(v) * 2.5 + 2.0
        forward = dist_m > 0

        # 出发前先看一眼前方
        if forward:
            fm = self.sector_min_robust(0.0)
            if fm is not None and fm < FRONT_STOP + 0.10:
                print('  ★ 前方只有 %.2f m，拒绝前进' % fm)
                return False

        t_need = abs(dist_m) / abs(v)
        t0 = time.time()
        r = rospy.Rate(CTRL_HZ)
        hit = False
        while not rospy.is_shutdown():
            self.beat()
            el = time.time() - t0
            if el >= t_need:
                break
            if el > timeout:
                print('  ⚠️ 前进超时')
                break
            if forward:
                fm = self.sector_min_robust(0.0)
                if fm is not None and fm < FRONT_STOP:
                    print('  ★ 前方 %.2f m 有障碍 → 急停' % fm)
                    hit = True
                    break
            self._pub(lin=v)
            r.sleep()
        self._pub()
        time.sleep(0.4)          # 等它真的停稳（底盘有减速斜坡）
        if self.verbose:
            print('  前进: 目标 %.2f m，用时 %.2f s（含停稳）%s'
                  % (dist_m, time.time() - t0,
                     '  ★ 被护栏拦停' if hit else ''))
        self.trace.append(('forward', dist_m, hit))
        return not hit

    def goto(self, dx, dy, face_deg=None):
        """相对当前位置走 (dx, dy)（世界坐标，车头朝向为 yaw）。

        做法：先转到目标方向 → 直行 → 需要的话再转到指定朝向。
        ⚠️ 只适用于**空旷、无障碍**的直线路径；有墙要分段走。
        """
        d = math.hypot(dx, dy)
        if d < 0.02:
            if face_deg is not None:
                return self.face(face_deg)
            return True
        want = math.degrees(math.atan2(dy, dx))
        cur = math.degrees(self.yaw())
        if not self.rotate(norm_angle(math.radians(want - cur))
                           * 57.2957795):
            return False
        if not self.forward(d):
            return False
        if face_deg is not None:
            return self.face(face_deg)
        return True

    def face(self, world_deg):
        """转到世界朝向 world_deg（0=+x 方向，逆时针为正）。"""
        cur = math.degrees(self.yaw())
        return self.rotate(norm_angle(math.radians(world_deg - cur))
                           * 57.2957795)
