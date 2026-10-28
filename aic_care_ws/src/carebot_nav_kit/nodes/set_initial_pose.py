#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
set_initial_pose.py — 启动时自动给 AMCL 设置初始位姿

用途
----
AMCL 必须知道机器人的初始位置才能开始定位。通常的做法是启动后在 rviz 里
用 "2D Pose Estimate" 手动点一下。但在以下场景手动操作很麻烦：
  · 机器人每次都从固定的出发区启动（比赛场景就是如此）
  · 需要反复重启导航做测试
  · 远程/无 GUI 环境

本节点可以在启动时自动发布初始位姿。

为什么这件事很重要
------------------
AMCL 的初始位姿给不准会导致：
  · 粒子云发散，定位迟迟不收敛（rviz 里看到一大片散点）
  · 导航目标全部失败，或机器人朝完全错误的方向走
  · 更糟的是"看起来收敛了但收敛到了错误位置"，机器人自信地走错路

用法
----
    rosrun carebot_nav_kit set_initial_pose.py --x 0.5 --y -0.3 --yaw 1.57

    # 或者由 navigation.launch 自动调用：
    roslaunch carebot_nav_kit navigation.launch initial_x:=0.5 initial_y:=-0.3 initial_yaw:=1.57

    # 从当前 TF 位置反向设置（把机器人当前位置"告诉"AMCL）
    rosrun carebot_nav_kit set_initial_pose.py --from-tf

参数
----
  --x --y --yaw      初始位姿（yaw 单位弧度）
  --from-tf          从 odom→base_footprint 的当前位姿推算
  --covariance       位置方差（默认 0.25 = 标准差 0.5m，表示"不太确定"）
  --delay            发布前等待秒数（默认 3，等 AMCL 起来）
  --repeat           重复发布次数（默认 3，防止 AMCL 还没订阅上）
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


def main():
    setup_stdout()
    fix_ros_logging()
    ap = argparse.ArgumentParser(description=u'设置 AMCL 初始位姿')
    ap.add_argument('--x', type=float, default=None)
    ap.add_argument('--y', type=float, default=None)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--from-tf', action='store_true',
                    help=u'从当前 TF 位姿推算初始位姿')
    ap.add_argument('--covariance', type=float, default=0.25,
                    help=u'位置方差（默认 0.25）')
    ap.add_argument('--yaw-covariance', type=float, default=0.0685,
                    help=u'朝向方差（默认 0.0685 ≈ 15°）')
    ap.add_argument('--delay', type=float, default=3.0)
    ap.add_argument('--repeat', type=int, default=3)
    ap.add_argument('--topic', default='/initialpose')
    ap.add_argument('--frame', default='map')
    args = ap.parse_args(rospy.myargv()[1:])

    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rospy.init_node('set_initial_pose', anonymous=True)

    if args.delay > 0:
        rospy.loginfo(u'[set_initial_pose] 等待 %.1f 秒让 AMCL 就绪…' % args.delay)
        rospy.sleep(args.delay)

    x, y, yaw = args.x, args.y, args.yaw

    if args.from_tf:
        rospy.loginfo(u'[set_initial_pose] 从 TF 读取当前位姿')
        try:
            import tf
            listener = tf.TransformListener()
            rospy.sleep(1.0)
            listener.waitForTransform('/' + args.frame, '/base_footprint',
                                      rospy.Time(0), rospy.Duration(5.0))
            trans, rot = listener.lookupTransform('/' + args.frame, '/base_footprint',
                                                  rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            x, y, yaw = trans[0], trans[1], euler[2]
            rospy.loginfo(u'[set_initial_pose] TF 位姿 = (%.3f, %.3f, %.3f rad)'
                          % (x, y, yaw))
        except Exception as exc:
            rospy.logerr(u'[set_initial_pose] 从 TF 读取失败：%s' % repr(exc))
            return 2

    if x is None or y is None:
        rospy.logerr(u'[set_initial_pose] 必须给出 --x --y，或使用 --from-tf')
        return 2

    pub = rospy.Publisher(args.topic, PoseWithCovarianceStamped, queue_size=1, latch=True)
    # 等订阅者连上（AMCL 可能刚启动）
    rospy.sleep(1.0)

    # 四元数：只绕 Z 轴
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    cov = [0.0] * 36
    cov[0] = args.covariance          # x
    cov[7] = args.covariance          # y
    cov[35] = args.yaw_covariance     # yaw
    # z / roll / pitch 给很小的值（2D 定位，这些方向本来就是确定的）
    cov[14] = 1e-6
    cov[21] = 1e-6
    cov[28] = 1e-6

    for i in range(args.repeat):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = args.frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = cov

        pub.publish(msg)
        rospy.loginfo(u'[set_initial_pose] 已发布初始位姿 (%.3f, %.3f, %.3f rad) [%d/%d]'
                      % (x, y, yaw, i + 1, args.repeat))
        rospy.sleep(0.5)

    rospy.loginfo(u'[set_initial_pose] 完成。请在 rviz 里确认粒子云已收敛。')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except ImportError:
        print(u'无法导入 rospy —— 请先 source ROS 环境与工作空间：')
        print(u'  source /opt/ros/kinetic/setup.bash')
        print(u'  source ~/omniWheelCareRobot/rosCode/devel/setup.bash')
        sys.exit(1)
