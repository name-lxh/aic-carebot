# -*- coding: utf-8 -*-
"""
odom_imu.py —— 干净的 odom 源（给 gmapping 用）  v2

## v1 错在哪（实测踩到）

v1 把 `/odom`（STM32 报的**绝对编码器值**）减去初值后平滑一下就用。
但那个值本身就带着巨大偏置（实测 -126 ~ -213 m），平滑只是让它"慢慢漂"。
结果 gmapping 被喂了错误的平移先验，**地图膨胀到 18.2 x 18.2 m**
（原点跑到 -14.55），而机器人在 3x3 场地里根本没走多远。

## v2 怎么做

**彻底不看 `/odom` 的位置。** 改成"命令速度积分"：

    · 订阅 `/cmd_vel`（就是我自己发的速度指令）
    · 按 v * dt 积分出平移
    · 只积**前进分量**（vy 不用 —— 实测底盘只实现了 vx / wz）
    · 加死区：|v| < 0.01 视为不动 → 原地旋转时平移**完全不动**
    · 航向仍然用 IMU（实测静止 30 s 只漂 0.03°，完全可信）

⚠️ 这不是"真实里程计"，是**开环速度积分**。好处是：
   · **不会漂到几百米**（没有绝对偏置）
   · 原地旋转时平移严格为 0（死区保证）
   · 幅值大致合理（比例误差 ~10~20%）
 对 gmapping 来说 —— 它主要靠**扫描匹配**定位，
 odom 只需要"连续、量级对、不跳"就够了。航向准是关键，由 IMU 保证。

## 发布

  · `/odom_clean` (nav_msgs/Odometry)
  · TF `odom` → `base_footprint`
"""
from __future__ import print_function

import math
import threading
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomImu(object):
    def __init__(self):
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.rate = float(rospy.get_param('~rate', 30.0))
        # 速度死区：小于这个就当不动（原地旋转时保证平移为 0）
        self.deadband = float(rospy.get_param('~deadband', 0.01))
        # 平移比例修正（开环积分通常会偏，按需微调）
        self.scale = float(rospy.get_param('~scale', 1.0))
        # 单周期最大位移，防跳变
        self.max_step = float(rospy.get_param('~max_step', 0.06))

        self._lock = threading.Lock()
        self._imu_yaw = None
        self._vx = 0.0            # 命令速度（兜底用）
        self._vx_fb = 0.0         # ★ 真实轮速（/feedback_vel）
        self._fb_stamp = 0.0
        self._x = 0.0
        self._y = 0.0
        self._t_last = None
        self._yaw0 = None
        self._n = 0

        self.tf_br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher('/odom_clean', Odometry, queue_size=10)

        rospy.Subscriber('/imu_data', Imu, self._on_imu, queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self._on_cmd, queue_size=20)
        # ★ 关键：平移用**轮子真实速度**积分。原来积分 /cmd_vel（命令值），
        #   轮子打滑/被卡时里程照样涨 -> AMCL 越走越飘，最后跳飞
        #   （2026-10-21 实车：跑到厨房后位姿跳到 (-0.87,-0.77)，全盘失效）。
        try:
            rospy.Subscriber('/feedback_vel', Twist, self._on_fb, queue_size=20)
            self.use_feedback = True
        except Exception:
            self.use_feedback = False

        rospy.Timer(rospy.Duration(1.0 / self.rate), self._tick)
        rospy.loginfo('[odom_imu v2] ready: %s -> %s @%.0f Hz  '
                      'deadband=%.3f scale=%.2f  (command integration, '
                      'IGNORES /odom)',
                      self.odom_frame, self.base_frame, self.rate,
                      self.deadband, self.scale)

    def _on_imu(self, m):
        with self._lock:
            self._imu_yaw = quat_to_yaw(m.orientation)

    def _on_cmd(self, t):
        with self._lock:
            self._vx = t.linear.x

    def _on_fb(self, t):
        with self._lock:
            self._vx_fb = t.linear.x
            self._fb_stamp = time.time()

    def _tick(self, _evt):
        now = rospy.Time.now()
        with self._lock:
            yaw = self._imu_yaw
            # 反馈速度新鲜（<0.5 s）就用它；否则退回命令值
            if self.use_feedback and (time.time() - self._fb_stamp) < 0.5:
                vx = self._vx_fb
            else:
                vx = self._vx
        if yaw is None:
            return
        if self._yaw0 is None:
            self._yaw0 = yaw
            self._t_last = now
            rospy.loginfo('[odom_imu v2] origin set, yaw0=%.2f deg',
                          math.degrees(yaw))
            return

        dt = (now - self._t_last).to_sec()
        self._t_last = now
        if dt <= 0.0 or dt > 0.5:      # 异常 dt 丢弃
            dt = 1.0 / self.rate

        # ★ 死区：原地旋转时 vx=0 → 平移严格不动
        v = 0.0 if abs(vx) < self.deadband else vx * self.scale

        # ★★★ 必须用**相对航向**（yaw - yaw0）做积分，不能用绝对 yaw！
        #
        #  原来这里是 `dx_w = v*dt*cos(yaw)`，用的是 IMU 的**绝对** yaw，
        #  而同一函数下面发布 odom 朝向时用的却是**相对**的 `yaw_rel = yaw - yaw0`。
        #  两者差一个 `yaw0`（odom_imu 启动瞬间的绝对航向），
        #  于是"发前进指令 -> odom 记录的位移方向"会整体旋转 `yaw0`。
        #
        #  实测（m16_tiny_move.py，指令前进 0.12 m）：
        #      位移 (-0.0146, -0.1131)  长度 0.1140 m   <- 长度正确
        #      位移方向 -97.4°，而 odom 报的车头朝向是 +0.1°
        #      夹角 -97.5° —— 而 odom_imu 启动时的 yaw0 正好约 -97.5°
        #  -> 完全对上，就是这个 bug。
        #
        #  后果非常严重：odom 说"机器人在往前走"，但位移方向被整体转了 yaw0。
        #  AMCL 看到的是机器人在**横着滑**，扫描匹配对不上，定位一路发散，
        #  move_base 就把车往错的方向带
        #  —— 这正是"目标在西北、车却往东南跑"的根因。
        yaw_rel = yaw - self._yaw0
        while yaw_rel > math.pi:
            yaw_rel -= 2 * math.pi
        while yaw_rel < -math.pi:
            yaw_rel += 2 * math.pi

        dx_w = v * dt * math.cos(yaw_rel)
        dy_w = v * dt * math.sin(yaw_rel)

        # 限幅防跳
        d = math.hypot(dx_w, dy_w)
        if d > self.max_step:
            k = self.max_step / d
            dx_w *= k
            dy_w *= k

        self._x += dx_w
        self._y += dy_w

        q = tf.transformations.quaternion_from_euler(0, 0, yaw_rel)
        self.tf_br.sendTransform((self._x, self._y, 0.0), q, now,
                                 self.base_frame, self.odom_frame)

        m = Odometry()
        m.header.stamp = now
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = self._x
        m.pose.pose.position.y = self._y
        m.pose.pose.orientation.x = q[0]
        m.pose.pose.orientation.y = q[1]
        m.pose.pose.orientation.z = q[2]
        m.pose.pose.orientation.w = q[3]
        # 协方差：告诉 gmapping "平移不太准，航向很准"
        cov = [0.25, 0.25, 0.0, 0.0, 0.0, 0.02]
        for i in range(6):
            m.pose.covariance[i * 7] = cov[i]
        self.pub.publish(m)

        self._n += 1
        if self._n % 300 == 0:
            rospy.loginfo('[odom_imu v2] x=%.3f y=%.3f yaw=%.1fdeg vx=%.3f',
                          self._x, self._y, math.degrees(yaw_rel), vx)


def main():
    rospy.init_node('odom_imu')
    OdomImu()
    rospy.spin()


if __name__ == '__main__':
    main()
