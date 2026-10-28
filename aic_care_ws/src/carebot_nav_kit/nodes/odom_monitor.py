#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
odom_monitor.py — 里程计健康监视

用途
----
建图和导航的精度**完全取决于里程计质量**。里程计有两个典型问题：

  1. **静止漂移**：机器人没动，但里程计位置/角度持续变化。
     后果：地图越建越歪；AMCL 粒子持续被推开，定位缓慢漂移。
     常见原因：IMU 零偏未标定、轮式编码器噪声、EKF 参数过于激进。

  2. **比例误差**：走 1m 但里程计报 0.85m。
     后果：地图尺度不对，导航目标系统性偏移。
     解决：用 calibrate_linear.py / calibrate_angular.py 标定系数。

本节点持续监视第 1 类问题，在漂移超阈值时告警，并周期性打印统计。

用法
----
    rosrun carebot_nav_kit odom_monitor.py
    rosrun carebot_nav_kit odom_monitor.py --drift-warn 0.02 --window 5.0
    rosrun carebot_nav_kit odom_monitor.py --topic /odometry/filtered
    rosrun carebot_nav_kit odom_monitor.py --calibrate   # 打印标定建议

参数
----
  --topic        监视的里程计话题，默认 /odom
  --window       静止判定窗口（秒），默认 5.0
  --motion-thresh 判定"在动"的位置变化阈值（m），默认 0.01
  --drift-warn   静止漂移告警阈值（m 或 rad，按窗口内增量），默认 0.02
  --report-period 周期统计打印间隔（秒），默认 15
"""

import argparse
import math
import sys
import time

try:
    unicode  # noqa: F821
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


def quat_to_yaw(q):
    """从四元数取 yaw。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class OdomMonitor(object):

    def __init__(self, args):
        self.args = args
        self.last = None          # 上一帧 (x, y, yaw, t)
        self.window_start = None  # 静止窗口起点
        self.state = 'unknown'    # unknown | still | moving
        self.drift_pos = 0.0
        self.drift_yaw = 0.0
        self.warn_count = 0
        self.total_dist = 0.0     # 累计行进距离
        self.total_yaw = 0.0      # 累计转角
        self.msg_count = 0
        self.t_first = None
        self.t_report = time.time()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        now = time.time()

        self.msg_count += 1
        if self.t_first is None:
            self.t_first = now

        if self.last is None:
            self.last = (p.x, p.y, yaw, now)
            self.window_start = (p.x, p.y, yaw, now)
            return

        lx, ly, lyaw, lt = self.last
        dx, dy = p.x - lx, p.y - ly
        dyaw = normalize_angle(yaw - lyaw)
        step_dist = math.hypot(dx, dy)

        self.total_dist += step_dist
        self.total_yaw += abs(dyaw)

        # --- 状态判定 ---
        moving = step_dist > self.args.motion_thresh / 10.0
        if moving:
            self.state = 'moving'
            self.window_start = (p.x, p.y, yaw, now)
            self.drift_pos = 0.0
            self.drift_yaw = 0.0
        else:
            self.state = 'still'
            if self.window_start is None:
                self.window_start = (p.x, p.y, yaw, now)
            sx, sy, syaw, st = self.window_start
            if now - st >= self.args.window:
                self.drift_pos = math.hypot(p.x - sx, p.y - sy)
                self.drift_yaw = abs(normalize_angle(yaw - syaw))
                self._check_drift(p, yaw, now)

        self.last = (p.x, p.y, yaw, now)

        # --- 周期报告 ---
        if now - self.t_report >= self.args.report_period:
            self._report(now)

    def _check_drift(self, p, yaw, now):
        a = self.args
        bad = False
        reason = []
        if self.drift_pos > a.drift_warn:
            bad = True
            reason.append(u'位置漂移 %.3f m' % self.drift_pos)
        if self.drift_yaw > a.drift_warn:
            bad = True
            reason.append(u'角度漂移 %.3f rad (%.1f°)' % (self.drift_yaw,
                                                        math.degrees(self.drift_yaw)))
        if bad:
            self.warn_count += 1
            # 同一窗口只提醒一次（靠重置 window_start）
            self.window_start = (p.x, p.y, yaw, now)
            import rospy
            rospy.logwarn(
                u'[odom_monitor] 检测到静止漂移：%s（%.1f 秒窗口内）'
                % (u'，'.join(reason), a.window))
            if self.warn_count == 1:
                rospy.logwarn(
                    u'[odom_monitor] 排查建议：'
                    u'(1) IMU 零偏未标定 —— 静止时看 /imu 的角速度是否非零；'
                    u'(2) 若刚启动就漂，等 IMU 预热 30 秒再建图；'
                    u'(3) 检查 EKF 的 imu0_config 是否误开了 roll/pitch；'
                    u'(4) 轮式里程计编码器噪声 —— 检查底盘串口是否有丢包')

    def _report(self, now):
        import rospy
        dur = now - (self.t_first or now)
        hz = self.msg_count / dur if dur > 0 else 0.0
        rospy.loginfo(
            u'[odom_monitor] 统计：%.1f Hz | 状态=%s | 累计行进 %.2f m / %.1f° | '
            u'当前漂移 pos=%.4f m yaw=%.4f rad | 漂移告警 %d 次'
            % (hz, self.state, self.total_dist, math.degrees(self.total_yaw),
               self.drift_pos, self.drift_yaw, self.warn_count))
        self.t_report = now


def main():
    setup_stdout()
    fix_ros_logging()
    import rospy

    # ⚠️⚠️ 必须用 `rospy.myargv()`，不能直接用 `sys.argv`。
    #
    # 实测崩溃（在 mapping.launch 里被 roslaunch 启动时）：
    #     odom_monitor.py: error: unrecognized arguments:
    #         __name:=odom_monitor __log:=/home/ub/.ros/log/.../odom_monitor-11.log
    #     [odom_monitor-11] process has died [exit code 2]
    #
    # 原因：roslaunch 会在命令行**追加** `__name:=` / `__log:=` 这类
    # ROS 专用参数（还可能有 remap 的 `foo:=bar`），
    # 而 argparse 不认识它们 → 直接报错退出。
    #
    # `rospy.myargv()` 会把这些 ROS 参数剥掉，只留下我们自己的参数。
    ap = argparse.ArgumentParser(description=u'里程计健康监视')
    ap.add_argument('--topic', default='/odom')
    ap.add_argument('--window', type=float, default=5.0)
    ap.add_argument('--motion-thresh', type=float, default=0.01)
    ap.add_argument('--drift-warn', type=float, default=0.02)
    ap.add_argument('--report-period', type=float, default=15.0)
    # myargv() 的第一个元素是脚本名，丢掉
    args = ap.parse_args(rospy.myargv()[1:])

    from nav_msgs.msg import Odometry

    rospy.init_node('odom_monitor', anonymous=True)
    mon = OdomMonitor(args)
    sub = rospy.Subscriber(args.topic, Odometry, mon.on_odom, queue_size=50)

    rospy.loginfo(u'[odom_monitor] 开始监视 %s' % args.topic)
    rospy.loginfo(u'  静止判定窗口 %.1f 秒，漂移告警阈值 %.3f'
                  % (args.window, args.drift_warn))
    rospy.loginfo(u'  ⚠️ 建图前请先让机器人静止 30 秒，观察是否有漂移告警')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    # 退出时给出总结
    if mon.msg_count == 0:
        print(u'\n[odom_monitor] 未收到任何里程计消息 —— 检查话题名是否正确：%s' % args.topic)
    else:
        print(u'\n[odom_monitor] 结束统计：')
        print(u'  累计行进      : %.2f m' % mon.total_dist)
        print(u'  累计转角      : %.1f°' % math.degrees(mon.total_yaw))
        print(u'  漂移告警次数  : %d' % mon.warn_count)
        if mon.warn_count == 0:
            print(u'  评价：里程计健康，可以放心建图/导航')
        else:
            print(u'  评价：存在静止漂移，建议先标定 IMU 零偏再建图')


if __name__ == '__main__':
    sys.exit(main() or 0)
