#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
calibrate_linear.py — 线速度/里程计比例标定

为什么必须做
------------
底盘收到的速度指令与它实际走的距离之间有系统误差，来源包括：
  · 轮径实际值与配置值不符（磨损、批次差异）
  · 编码器分辨率/减速比配置不准
  · 地面打滑

后果：地图尺度不对。走 1m 里程计只报 0.85m，那么地图上所有东西都会
被"拉伸"15%，导航到点位时系统性偏移 —— 而且这种误差**不会随定位收敛而消失**。

怎么用
------
  1. 把机器人放在地上，前方留出至少 2.5m 直线空间，地面干净不打滑
  2. 用卷尺在起点做个标记，量出 2.0m 的位置做个终点标记
  3. 运行本脚本：
       rosrun carebot_nav_kit calibrate_linear.py
  4. 机器人会直行，同时打印"里程计报告走了多远"
  5. 机器人停下后，用卷尺量出**实际**走了多远
  6. 脚本会算出修正系数，填入 base.yaml 和底盘配置

参数
----
  --distance      测试距离（米），默认 2.0
  --speed         测试速度（m/s），默认 0.15（慢速减少打滑）
  --topic         里程计话题，默认 /odom
  --axis          x | y（全向底盘可测横移），默认 x
  --dry-run       只打印计划不发指令
  --cmd-topic     速度指令话题，默认 /cmd_vel
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
    """跨 Python 2/3 的输入函数。"""
    if sys.version_info[0] == 2:
        return raw_input(prompt)  # noqa: F821
    return input(prompt)


class LinearCalibrator(object):

    def __init__(self, args):
        self.args = args
        self.start_pos = None
        self.last_pos = None
        self.done = False

    def on_odom(self, msg):
        p = msg.pose.pose.position
        # 用 x/y 分量（取决于测试轴）
        if self.args.axis == 'y':
            cur = p.y
        else:
            cur = p.x

        if self.start_pos is None:
            self.start_pos = cur
            self.last_pos = cur
            return
        self.last_pos = cur

    def travelled(self):
        if self.start_pos is None or self.last_pos is None:
            return 0.0
        return abs(self.last_pos - self.start_pos)


def main():
    setup_stdout()
    ap = argparse.ArgumentParser(description=u'线速度/里程计比例标定')
    ap.add_argument('--distance', type=float, default=2.0)
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--topic', default='/odom')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--axis', choices=['x', 'y'], default='x')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--timeout', type=float, default=60.0)
    args = ap.parse_args(rospy.myargv()[1:])

    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    rospy.init_node('calibrate_linear', anonymous=True)

    print(u'\n' + u'=' * 68)
    print(u'  线速度标定')
    print(u'=' * 68)
    print(u'  测试轴   : %s' % args.axis)
    print(u'  目标距离 : %.2f m' % args.distance)
    print(u'  测试速度 : %.2f m/s' % args.speed)
    print(u'  里程计   : %s' % args.topic)
    print(u'=' * 68)

    print(u'\n\033[33;1m⚠️  准备检查：\033[0m')
    print(u'  1. 机器人前方至少 %.1f m 无障碍' % (args.distance + 0.5))
    print(u'  2. 已在起点做好地面标记')
    print(u'  3. 已在 %.1f m 处做好终点标记（便于测实际距离）' % args.distance)
    print(u'  4. 地面干燥不打滑')

    if args.dry_run:
        print(u'\n[--dry-run] 只打印计划，不实际运动')
        return 0

    print(u'\n按回车开始标定（Ctrl+C 取消）...')
    try:
        _input()
    except (KeyboardInterrupt, EOFError):
        print(u'\n已取消')
        return 1

    cal = LinearCalibrator(args)
    sub = rospy.Subscriber(args.topic, Odometry, cal.on_odom, queue_size=20)
    pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=5)

    print(u'\n等待里程计数据…')
    t0 = time.time()
    while cal.start_pos is None and time.time() - t0 < 10.0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    if cal.start_pos is None:
        print(u'\033[31;1m✘ 收不到里程计数据（%s）\033[0m' % args.topic)
        print(u'  检查：rostopic hz %s' % args.topic)
        return 1

    print(u'起点里程计读数 = %.4f' % cal.start_pos)
    print(u'\n开始直行…\n')

    twist = Twist()
    if args.axis == 'y':
        twist.linear.y = args.speed
    else:
        twist.linear.x = args.speed

    rate = rospy.Rate(20)
    t_start = time.time()
    last_print = 0.0

    while not rospy.is_shutdown():
        travelled = cal.travelled()
        elapsed = time.time() - t_start

        if elapsed - last_print > 0.3:
            last_print = elapsed
            print(u'\r  里程计报告：%.3f m / %.2f m  (%.0f%%)   用时 %.1f s'
                  % (travelled, args.distance, 100.0 * travelled / args.distance, elapsed)),
            sys.stdout.flush()

        if travelled >= args.distance:
            break
        if elapsed > args.timeout:
            print(u'\n\033[33m⚠️  超时（%.0f 秒），里程计只报告了 %.3f m\033[0m'
                  % (args.timeout, travelled))
            print(u'  可能原因：机器人没动 / 速度太慢 / 里程计不更新')
            break

        pub.publish(twist)
        rate.sleep()

    # 停车
    pub.publish(Twist())
    rospy.sleep(1.0)

    odom_dist = cal.travelled()
    print(u'\n\n机器人已停止。里程计报告走了 %.4f m。' % odom_dist)

    if odom_dist < 0.05:
        print(u'\033[31;1m✘ 几乎没动，标定无法进行\033[0m')
        return 1

    print(u'\n' + u'-' * 68)
    print(u'现在用卷尺量出机器人**实际**走了多远（从起点标记到当前位置）。')
    print(u'-' * 68)
    try:
        raw = _input(u'实际距离 (m): ')
    except (KeyboardInterrupt, EOFError):
        print(u'\n已取消')
        return 1

    try:
        actual = float(raw.strip())
    except ValueError:
        print(u'\033[31;1m✘ 输入不是数字\033[0m')
        return 1

    if actual <= 0.01:
        print(u'\033[31;1m✘ 实际距离必须大于 0\033[0m')
        return 1

    # 修正系数 = 实际 / 报告
    scale = actual / odom_dist
    error_pct = (odom_dist - actual) / actual * 100.0

    print(u'\n' + u'=' * 68)
    print(u'  标定结果')
    print(u'=' * 68)
    print(u'  里程计报告 : %.4f m' % odom_dist)
    print(u'  实际测量   : %.4f m' % actual)
    print(u'  误差       : %+.2f%%' % error_pct)
    print()
    print(u'  \033[1m修正系数 linear_scale = %.6f\033[0m' % scale)
    print(u'=' * 68)
    print()
    print(u'  如何应用：')
    print(u'  1) 编辑 config/robot/base.yaml：')
    print(u'       calibration:')
    print(u'         linear_scale: %.6f' % scale)
    print()
    print(u'  2) 若底盘驱动自己带修正参数（如 ros_arduino_python 的')
    print(u'     linear_scale_correction），把该参数乘上 %.6f' % scale)
    print()
    print(u'  3) 重新跑一次标定验证：修正后误差应在 ±2% 以内')
    print()

    if abs(error_pct) < 2.0:
        print(u'  \033[32;1m✔ 当前误差已在 ±2% 内，其实可以不修正\033[0m')
    else:
        print(u'  \033[33;1m⚠️  误差较大，建议应用上面的修正系数后重新验证\033[0m')
    print()

    return 0


if __name__ == '__main__':
    sys.exit(main())
