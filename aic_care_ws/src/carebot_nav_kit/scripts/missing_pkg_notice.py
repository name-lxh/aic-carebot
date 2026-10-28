#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
missing_pkg_notice.py — 缺失依赖包的友好提示节点

为什么需要它
------------
`carebot_nav_kit` 的设计目标是"不硬依赖组委会的包"，让整套东西能打包到
不同配置的机器上跑。但 `$(find some_pkg)` 在包不存在时会直接抛异常，
导致整个 launch 崩掉。

所以在 include 外面套了 `$(eval find(...) is not None)` 判断，
包不存在时改启动本节点 —— 它只打印一条清楚的提示，然后退出。

这样：
  · 启动不会中断，其他子系统照常工作
  · 用户能看到明确的"缺什么、怎么装"，而不是一句 Resource not found

用法（由 launch 调用，不需要手动跑）：
    rosrun carebot_nav_kit missing_pkg_notice.py <包名> <用途说明>
"""

import sys

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


# 各缺失包的针对性建议
HINTS = {
    'base_control': u'''\
      这是组委会提供的 STM32 底盘驱动包（实验室实车用）。
      获取方式：把组委会源码里的 base_control 包放到工作空间 src/ 下再 catkin_make。
      如果你只是想在开发机上验证导航逻辑、不看真实底盘，可以忽略这条提示，
      改用：driver:=none''',

    'ros_arduino_python': u'''\
      这是备选底盘驱动（Arduino/兼容板）。
      组委会源码里有：ros_arduino_bridge/ros_arduino_python。
      忽略本提示可改用：driver:=none''',

    'lslidar_driver': u'''\
      这是镭神 LSLIDAR M10 的驱动包（实验室实车用）。
      获取方式：组委会源码里的 lslidar_driver（或 lsm10_v2）。
      替代方案：如果用 RPLIDAR A1M8，改用 laser_driver:=rplidar_a2''',

    'ls01d_lidar': u'''\
      这是 ls01d 系列雷达驱动。
      组委会源码里有：ls01d_lidar。
      替代方案：laser_driver:=rplidar_a2（RPLIDAR）或 lslidar_m10''',

    'rplidar_ros': u'''\
      这是 RPLIDAR 的 ROS 驱动包。
      安装：sudo apt install ros-<distro>-rplidar-ros
      注意 Kinetic/Melodic 上是 rplidar_ros，默认设备名 /dev/ttyUSB0，
      建议配 udev 规则固定为 /dev/rplidar。''',

    'fdilink_ahrs': u'''\
      这是 fdilink AHRS 惯性单元的驱动（实验室实车用）。
      获取方式：组委会源码里的 fdilink_ahrs。
      替代方案：若用 art_imu_01 型号，改用 imu_driver:=art_imu；
               若不用 IMU，加 start_imu:=false''',

    'art_imu_01': u'''\
      这是 art_imu_01 型号 IMU 的驱动。
      组委会源码里有：art_imu_01。
      替代方案：imu_driver:=fdilink 或 start_imu:=false''',

    'astra_launch': u'''\
      这是奥比中光 Astra 深度相机的启动包。
      获取方式：组委会源码里的 astra_launch / ros_astra_camera。
      相机与导航无关，缺了不影响建图导航（人脸/二维码功能会受影响）。''',
}

DEFAULT_HINT = u'''\
      该包不在 ROS 路径里。
      检查方式：rospack find <包名>
      如果你不需要这个子系统，可以在 launch 参数里关掉它。'''


def main():
    setup_stdout()
    fix_ros_logging()
    if len(sys.argv) < 2:
        pkg = u'(未指定)'
        purpose = u''
    else:
        pkg = unicode(sys.argv[1])
        purpose = unicode(sys.argv[2]) if len(sys.argv) > 2 else u''

    line = u'=' * 70
    msg = u'''
%s
  ⚠️  依赖包缺失：%s
%s

  用途：%s

%s

  这不会中断启动 —— 其他子系统会照常运行。
%s
''' % (line, pkg, line, purpose or u'(未说明)',
       HINTS.get(pkg, DEFAULT_HINT), line)

    # 用 print 而不是 rospy.logwarn，保证在 ROS 还没完全起来时也能看到
    try:
        print(msg.encode('utf-8') if isinstance(msg, unicode) else msg)
    except Exception:
        print(msg)

    # 如果 ROS 已经初始化，再用 logwarn 记一条
    try:
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node('missing_pkg_notice', anonymous=True, disable_signals=True)
        rospy.logwarn(u'依赖包缺失：%s（%s）' % (pkg, purpose))
    except Exception:
        pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
