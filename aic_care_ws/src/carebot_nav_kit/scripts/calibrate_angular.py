#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
calibrate_angular.py — 角速度/朝向标定

为什么必须做
------------
朝向误差比距离误差更致命：
  · 距离误差 5%  -> 走 2m 偏 10cm，勉强能忍
  · 朝向误差 5%  -> 转 180° 实际转了 171°，差 9°，
                    下一个点位就完全对不上
更麻烦的是朝向误差会**累积**：连续转几次之后偏差越滚越大。

AMCL 能修正一部分，但如果里程计朝向误差太大，粒子云会持续被推开，
表现为"定位一直不收敛"或"收敛后又慢慢漂走"。

怎么用
------
  1. 把机器人放在场地中央，周围留出至少 0.8m 半径空间
  2. 在地面上用胶带标出机器人初始朝向
  3. 运行本脚本：
       rosrun carebot_nav_kit calibrate_angular.py
  4. 机器人会原地转 360°（可配置转 2 圈提高精度）
  5. 停下后，观察机器人实际朝向与初始标记的偏差
  6. 输入实际转过的角度，脚本算出修正系数

参数
----
  --turns         转几圈，默认 1（建议 2 圈，误差摊薄后更准）
  --speed         角速度 rad/s，默认 0.4（慢速减少打滑）
  --topic         里程计话题，默认 /odom
  --direction     ccw | cw，默认 ccw（逆时针）
  --cmd-topic     速度指令话题，默认 /cmd_vel
  --timeout       超时秒数，默认 120
"""

import argparse
import math
import sys
import time


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

try:
    unicode  # noqa: F821
except NameError:
    unicode = str  # noqa: A001


def _input(prompt=u''):
    if sys.version_info[0] == 2:
        return raw_input(prompt)  # noqa: F821
    return input(prompt)


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class AngularCalibrator(object):

    def __init__(self, args):
        self.args = args
        self.prev_yaw = None
        self.accum = 0.0        # 累积转角（带符号）
        self.started = False

    def on_odom(self, msg):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self.prev_yaw is None:
            self.prev_yaw = yaw
            return
        delta = normalize_angle(yaw - self.prev_yaw)
        # 双向累积（防止单帧跳变被误当成长转角）
        if abs(delta) < math.radians(30):
            self.accum += delta
        self.prev_yaw = yaw

    def degrees(self):
        return math.degrees(abs(self.accum))


def main():
    setup_stdout()
    ap = argparse.ArgumentParser(description=u'角速度/朝向标定')
    ap.add_argument('--turns', type=float, default=1.0)
    ap.add_argument('--speed', type=float, default=0.4)
    ap.add_argument('--topic', default='/odom')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--direction', choices=['ccw', 'cw'], default='ccw')
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args(rospy.myargv()[1:])

    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    rospy.init_node('calibrate_angular', anonymous=True)

    target_deg = 360.0 * args.turns
    target_rad = math.radians(target_deg)

    print(u'\n' + u'=' * 68)
    print(u'  角速度标定')
    print(u'=' * 68)
    print(u'  转圈数   : %.1f（%.0f°）' % (args.turns, target_deg))
    print(u'  角速度   : %.2f rad/s' % args.speed)
    print(u'  方向     : %s' % (u'逆时针' if args.direction == 'ccw' else u'顺时针'))
    print(u'  里程计   : %s' % args.topic)
    print(u'=' * 68)

    print(u'\n\033[33;1m⚠️  准备检查：\033[0m')
    print(u'  1. 周围至少 %.1f m 半径无障碍' % (0.5 + 0.3))
    print(u'  2. 已在地面标出机器人**初始朝向**（用胶带贴一条线）')
    print(u'  3. 机器人重心稳定，地毯/电线不会绊住轮子')
    print(u'\n  提示：用 %.0f 圈比 1 圈更准（误差被摊薄）' % 2)

    print(u'\n按回车开始（Ctrl+C 取消）...')
    try:
        _input()
    except (KeyboardInterrupt, EOFError):
        print(u'\n已取消')
        return 1

    cal = AngularCalibrator(args)
    sub = rospy.Subscriber(args.topic, Odometry, cal.on_odom, queue_size=50)
    pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=5)

    print(u'\n等待里程计数据…')
    t0 = time.time()
    while cal.prev_yaw is None and time.time() - t0 < 10.0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    if cal.prev_yaw is None:
        print(u'\033[31;1m✘ 收不到里程计数据（%s）\033[0m' % args.topic)
        return 1

    print(u'开始旋转…\n')

    twist = Twist()
    twist.angular.z = args.speed if args.direction == 'ccw' else -args.speed

    rate = rospy.Rate(20)
    t_start = time.time()
    last_print = 0.0

    while not rospy.is_shutdown():
        cur_deg = cal.degrees()
        elapsed = time.time() - t_start

        if elapsed - last_print > 0.3:
            last_print = elapsed
            print(u'\r  里程计累积转角：%7.1f° / %.0f°  (%.0f%%)   用时 %.1f s'
                  % (cur_deg, target_deg, 100.0 * cur_deg / target_deg, elapsed)),
            sys.stdout.flush()

        if cur_deg >= target_deg:
            break
        if elapsed > args.timeout:
            print(u'\n\033[33m⚠️  超时（%.0f 秒），只转了 %.1f°\033[0m'
                  % (args.timeout, cur_deg))
            print(u'  可能原因：角速度太小 / 底盘不动 / 里程计朝向不更新')
            break

        pub.publish(twist)
        rate.sleep()

    pub.publish(Twist())
    rospy.sleep(1.0)

    odom_deg = cal.degrees()
    print(u'\n\n已停止。里程计报告转了 %.2f°（目标 %.0f°）。' % (odom_deg, target_deg))

    if odom_deg < 10.0:
        print(u'\033[31;1m✘ 几乎没转，标定无法进行\033[0m')
        return 1

    print(u'\n' + u'-' * 68)
    print(u'现在观察机器人的**实际朝向**，与开始时贴的初始朝向标记比较。')
    print(u'量出实际转过的角度（顺时针偏差填正数，逆时针偏差填正数）。')
    print(u'-' * 68)
    try:
        raw = _input(u'实际转过的角度 (度): ')
    except (KeyboardInterrupt, EOFError):
        print(u'\n已取消')
        return 1

    try:
        actual_deg = float(raw.strip())
    except ValueError:
        print(u'\033[31;1m✘ 输入不是数字\033[0m')
        return 1

    if actual_deg <= 1.0:
        print(u'\033[31;1m✘ 实际角度必须大于 1°\033[0m')
        return 1

    scale = actual_deg / odom_deg
    error_pct = (odom_deg - actual_deg) / actual_deg * 100.0

    print(u'\n' + u'=' * 68)
    print(u'  标定结果')
    print(u'=' * 68)
    print(u'  里程计报告 : %.2f°' % odom_deg)
    print(u'  实际测量   : %.2f°' % actual_deg)
    print(u'  误差       : %+.2f%%' % error_pct)
    print()
    print(u'  \033[1m修正系数 angular_scale = %.6f\033[0m' % scale)
    print(u'=' * 68)
    print()
    print(u'  如何应用：')
    print(u'  1) 编辑 config/robot/base.yaml：')
    print(u'       calibration:')
    print(u'         angular_scale: %.6f' % scale)
    print()
    print(u'  2) 若底盘驱动带角度修正参数（如 ros_arduino_python 的')
    print(u'     angular_scale_correction），把该参数乘上 %.6f' % scale)
    print()
    print(u'  3) 重新跑一次验证：修正后误差应在 ±2% 以内')
    print()

    if abs(error_pct) < 2.0:
        print(u'  \033[32;1m✔ 当前误差已在 ±2% 内，其实可以不修正\033[0m')
    elif abs(error_pct) > 10.0:
        print(u'  \033[31;1m⚠️  误差超过 10%%，这会导致严重的导航偏移，务必修正！\033[0m')
        print(u'      另外请检查：轮径/轮距参数是否正确、IMU 零偏是否标定')
    else:
        print(u'  \033[33;1m⚠️  误差较大，建议应用修正系数后重新验证\033[0m')
    print()

    return 0


if __name__ == '__main__':
    sys.exit(main())
