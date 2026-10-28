#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
voice_bridge.py — 智慧养老护理赛题：语音交互桥接节点

为什么需要这一层
----------------
组委会提供**两个**语音功能包，它们的话题名互不匹配，直接同时启动是接不通的：

  | 包 | 发布 | 订阅 |
  |---|---|---|
  | `xf_mic_asr_offline` | `voice_words`(下划线) | — |
  | `robot_voice/iat_publish` | `voiceWords`(驼峰) | `voiceWakeup`(驼峰) |
  | `robot_voice/tts_subscribe` | — | `voice_words`(下划线) |
  | `iot_modules/IOT_net` | `/voice_tts/tts_topic` | — |

  ⇒ `robot_voice/iat_publish` 发布的 `voiceWords` **没有任何节点订阅**；
     `tts_subscribe` 订阅的 `voice_words` **没有任何节点发布**。链路是断的。

本节点做四件事
--------------
  1. **话题名归一化**：把 `voiceWords` / `voice_words` / `/voice_tts/tts_topic`
     统一成 `/voice_words`（识别结果）与 `/tts_say`（要朗读的文本）。
  2. **可靠的 TTS 播放**：参考实现用 `popen("play tts_sample.wav")` + 固定文件名 +
     `sleep(1)`。连续播报会互相覆盖文件名、且不等播放完成就返回。
     本节点改为：唯一文件名 + 播放队列 + **等待播放真正结束**（子进程 wait）。
  3. **语音讲解**：提供 `/voice/announce` 服务与 `/voice/say` 便捷订阅，
     任务状态机每到一个环节就调用，满足赛题「语音讲解」要求。
  4. **声源定位**：转发麦克风阵列的唤醒方位角（`/mic/awake/angle`，1° 分辨率），
     供「主动转向说话人」创意功能使用。

订阅
----
  /voice/say          std_msgs/String   要朗读的文本（推荐入口，自动入队）
  /tts_say            std_msgs/String   同上（兼容用）
  /voiceWords         std_msgs/String   来自 robot_voice/iat_publish 的识别结果
  voice_words         std_msgs/String   来自 xf_mic_asr_offline 的识别结果
  /mic/awake/angle    std_msgs/Int32    唤醒声源方位角（度）

发布
----
  /voice_words        std_msgs/String   归一化后的识别结果（下游统一订阅这个）
  /voice/wakeup       std_msgs/Bool     唤醒事件
  /voice/awake_angle  std_msgs/Int32    唤醒方位角（转发）
  /voice/status       std_msgs/String   人类可读状态（2 号机位展示）

服务
----
  /voice/announce           std_srvs/Trigger   请求 message 为文本，朗读并等待完成
  /voice/listen_once        std_srvs/Trigger   触发一次离线命令词识别，返回识别文本
  /voice/set_awake_word     std_srvs/Trigger   请求 message 为新唤醒词
  /voice/led                std_srvs/Trigger   请求 message 为 LED id（99 = 关灯）
  /voice/stop               std_srvs/Trigger   清空播报队列并停止当前播放

参数
----
  ~play_cmd       播放器命令，默认 "aplay -q"；如用 PulseAudio 可改 "paplay"
  ~tts_tmp_dir    临时 wav 目录，默认 /tmp/carebot_tts
  ~announce_gap   两次播报之间的最小间隔（秒），默认 0.2
  ~tts_bin        产生 wav 的可执行文件，默认 robot_voice/tts_subscribe
                  （若为空则退化为直接调用系统 festival/espeak，见 _fallback_tts）
  ~enable_tts     默认 true；false 时只发状态不朗读（纯调试用）
  ~echo_to_screen 默认 true；把播报文本也发到 /voice/status
"""

import sys
import os
import subprocess
import threading
import time

import rospy
from std_msgs.msg import Bool, Int32, String
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
    """统一转 UTF-8 字节串（Py2 下 ROS String 不接受 unicode）。"""
    if isinstance(text, unicode):
        return text.encode('utf-8')
    return text


def _to_unicode(text):
    if isinstance(text, bytes):
        return text.decode('utf-8', 'replace')
    if isinstance(text, unicode):
        return text
    return unicode(text)


class VoiceBridge(object):

    def __init__(self):
        self.play_cmd = rospy.get_param('~play_cmd', 'aplay -q')
        self.tts_tmp_dir = rospy.get_param('~tts_tmp_dir', '/tmp/carebot_tts')
        self.announce_gap = float(rospy.get_param('~announce_gap', 0.2))
        self.tts_bin = rospy.get_param('~tts_bin', '')     # 空 = 用内置合成器
        self.enable_tts = bool(rospy.get_param('~enable_tts', True))
        self.echo_to_screen = bool(rospy.get_param('~echo_to_screen', True))

        if not os.path.isdir(self.tts_tmp_dir):
            try:
                os.makedirs(self.tts_tmp_dir)
            except OSError:
                pass

        # 播报队列
        self.queue = []
        self.queue_lock = threading.Lock()
        self.queue_cv = threading.Condition(self.queue_lock)
        self.current_proc = None
        self.stop_flag = False
        self._seq = 0

        # 发布者
        self.pub_words = rospy.Publisher('/voice_words', String, queue_size=20)
        self.pub_wakeup = rospy.Publisher('/voice/wakeup', Bool, queue_size=5)
        self.pub_angle = rospy.Publisher('/voice/awake_angle', Int32, queue_size=5, latch=True)
        self.pub_status = rospy.Publisher('/voice/status', String, queue_size=10, latch=True)

        # 兼容桥：把各种别名话题汇入统一出口
        self.sub_say = rospy.Subscriber('/voice/say', String, self.on_say, queue_size=20)
        self.sub_tts_say = rospy.Subscriber('/tts_say', String, self.on_say, queue_size=20)
        self.sub_tts_topic = rospy.Subscriber('/voice_tts/tts_topic', String, self.on_say, queue_size=20)

        self.sub_words_camel = rospy.Subscriber('/voiceWords', String, self.on_words, queue_size=20)
        self.sub_words_snake = rospy.Subscriber('/voice_words', String, self.on_words, queue_size=20)
        self.sub_awake_flag = rospy.Subscriber('/awake_flag', Int32, self.on_awake_flag, queue_size=5)
        self.sub_awake_angle = rospy.Subscriber('/mic/awake/angle', Int32, self.on_awake_angle, queue_size=5)

        # 服务
        self.srv_announce = rospy.Service('/voice/announce', Trigger, self.on_announce)
        self.srv_listen = rospy.Service('/voice/listen_once', Trigger, self.on_listen_once)
        self.srv_set_word = rospy.Service('/voice/set_awake_word', Trigger, self.on_set_awake_word)
        self.srv_led = rospy.Service('/voice/led', Trigger, self.on_led)
        self.srv_stop = rospy.Service('/voice/stop', Trigger, self.on_stop)

        # 延迟探测可用的 TTS 后端（此时 tts_subscribe 可能还没起来）
        rospy.Timer(rospy.Duration(5.0), self._probe_tts, oneshot=True)

        # 播放线程
        self.player = threading.Thread(target=self._player_loop)
        self.player.daemon = True
        self.player.start()

        self._set_status(u'语音桥接就绪（播放器：%s）' % self.play_cmd)
        self.say(u'语音系统启动完成')

    # ------------------------------------------------------------ 后端探测
    def _probe_tts(self, _evt):
        """探测讯飞离线 TTS 节点是否在跑，决定用哪条合成路径。"""
        topics = [t for t in rospy.get_published_topics()]
        names = [t[0] for t in topics]
        has_tts_node = any('robot_voice' in str(n) for n in names) or \
            self._node_exists('/TextToSpeech')
        if has_tts_node:
            self._set_status(u'检测到机器人端 TTS 节点，播报将走讯飞离线合成')
        else:
            self._set_status(u'未检测到讯飞 TTS 节点，播报将使用备用合成器（espeak/festival）')

    @staticmethod
    def _node_exists(name):
        try:
            import rosgraph
            master = rosgraph.Master('/voice_bridge_probe')
            return name in master.getSystemState()[0]
        except Exception:
            return False

    # ------------------------------------------------------------ 合成与播放
    def _fallback_tts(self, text, out_path):
        """备用合成：espeak / festival。仅在讯飞 TTS 不可用时使用。

        注意：备用合成器对中文支持有限（espeak 需 mbrola 中文语音包），
        所以这只是在紧急情况下的保底，正常比赛必须用讯飞离线 TTS。
        """
        for cmd in (
            # 中文用「慢一点 + 加词间隔 + 降音高」，否则 espeak 的普通话连读很含糊
            ['espeak-ng', '-v', 'zh', '-s', '120', '-g', '4', '-p', '55', '-w', out_path, text],
            ['espeak', '-v', 'zh', '-s', '120', '-g', '4', '-p', '55', '-w', out_path, text],
            ['festival', '--tts'],
        ):
            try:
                if cmd[0] == 'festival':
                    continue
                subprocess.call(cmd)
                if os.path.isfile(out_path) and os.path.getsize(out_path) > 0:
                    return True
            except OSError:
                continue
        return False

    def _synthesize(self, text, out_path):
        """把文本合成为 wav 文件。

        首选：外部 TTS 命令行（用 ~tts_bin 指定，例如封装好的讯飞离线合成器）
        次选：espeak-ng / espeak / festival

        关于讯飞离线 TTS 的集成方式说明
        ------------------------------
        `robot_voice/tts_subscribe.cpp` 是一个**常驻 ROS 节点**，它订阅
        `voice_words`，每次收到文本就合成到**当前工作目录下的固定文件名**
        `tts_sample.wav`，然后 `popen("play tts_sample.wav")` 播放，并 `sleep(1)`。

        这个设计有三个问题，导致它不能作为本项目的播报通道：
          1. 固定文件名 —— 连续播报会互相覆盖；
          2. `sleep(1)` 不等播放结束 —— 状态机会在语音还没播完时就继续走；
          3. 没有任何回执 —— 上层无法知道播报何时完成。

        因此本节点**不复用 tts_subscribe 节点**，而是直接调用讯飞离线合成能力：
        推荐做法是用 `robot_voice` 的 libmsc 写一个一次性命令行合成器，
        通过 ~tts_bin 参数挂进来（参见 scripts/xf_tts_cli.cpp 与 README）。
        在它到位之前，退化到 espeak，保证链路可跑通。
        """
        if self.tts_bin:
            try:
                subprocess.call([self.tts_bin, text, out_path])
                if os.path.isfile(out_path) and os.path.getsize(out_path) > 0:
                    return True
            except OSError as exc:
                rospy.logwarn_throttle(10.0, '[voice_bridge] tts_bin 调用失败: %s', exc)
        return self._fallback_tts(text, out_path)

    def _play(self, wav_path):
        """播放 wav 并**等待播放结束**（修复参考实现 sleep(1) 不等播放的缺陷）。"""
        cmd = self.play_cmd.split() + [wav_path]
        try:
            self.current_proc = subprocess.Popen(cmd)
            self.current_proc.wait()
        except OSError as exc:
            rospy.logwarn('[voice_bridge] 播放失败 %s: %s', self.play_cmd, exc)
        finally:
            self.current_proc = None

    def _player_loop(self):
        """串行播放队列，保证播报不重叠、不漏播。"""
        while not rospy.is_shutdown():
            with self.queue_cv:
                while not self.queue and not rospy.is_shutdown():
                    self.queue_cv.wait(0.5)
                if rospy.is_shutdown():
                    return
                text = self.queue.pop(0)

            if self.stop_flag:
                self.stop_flag = False
                continue

            self._seq += 1
            wav_path = os.path.join(self.tts_tmp_dir, 'say_%d_%d.wav' % (os.getpid(), self._seq))

            if self.enable_tts:
                if self._synthesize(text, wav_path):
                    self._play(wav_path)
                else:
                    rospy.logwarn('[voice_bridge] 无可用的 TTS 合成器，文本未朗读：%s', _u8(text))
                try:
                    if os.path.isfile(wav_path):
                        os.remove(wav_path)
                except OSError:
                    pass

            if self.echo_to_screen:
                self._set_status(u'播报：%s' % text)
            time.sleep(self.announce_gap)

    def say(self, text, blocking=False, timeout=30.0):
        """把文本加入播报队列。blocking=True 时等待播放完成。"""
        text = _to_unicode(text).strip()
        if not text:
            return
        with self.queue_cv:
            self.queue.append(text)
            self.queue_cv.notify()
        if blocking:
            deadline = time.time() + timeout
            while time.time() < deadline and not rospy.is_shutdown():
                with self.queue_lock:
                    if text not in self.queue and self.current_proc is None:
                        return
                time.sleep(0.1)

    # ------------------------------------------------------------ 回调
    def on_say(self, msg):
        self.say(msg.data)

    def on_words(self, msg):
        text = _to_unicode(msg.data).strip()
        if not text:
            return
        # 防止自己播报的文本回环（参考实现会把播报文本也发到识别话题上）
        if text in (u'温度', u'湿度', u'光强', u'光照', u'烟雾'):
            return
        self.pub_words.publish(String(data=_u8(text)))
        self._set_status(u'识别到：%s' % text)

    def on_awake_flag(self, msg):
        if msg.data:
            self.pub_wakeup.publish(Bool(data=True))
            self._set_status(u'已唤醒')

    def on_awake_angle(self, msg):
        self.pub_angle.publish(Int32(data=msg.data))
        self._set_status(u'唤醒声源方位角：%d°' % msg.data)

    # ------------------------------------------------------------ 服务
    def on_announce(self, req):
        text = _to_unicode(getattr(req, 'message', '') or '').strip()
        if not text:
            return TriggerResponse(success=False, message=_u8(u'请提供要朗读的文本'))
        self.say(text, blocking=True)
        return TriggerResponse(success=True, message=_u8(u'已播报'))

    def on_listen_once(self, _req):
        """触发一次离线命令词识别，返回识别文本。"""
        try:
            from xf_mic_asr_offline.srv import Get_Offline_Result_srv
        except ImportError:
            return TriggerResponse(
                success=False,
                message=_u8(u'缺少 xf_mic_asr_offline 的 srv 定义，请先 catkin_make 编译该包'))

        try:
            rospy.wait_for_service('/get_offline_recognise_result_srv', timeout=3.0)
        except rospy.ROSException:
            return TriggerResponse(
                success=False,
                message=_u8(u'离线识别服务不可用，请确认已启动 '
                            u'xf_mic_asr_offline/voice_control.launch'))

        try:
            proxy = rospy.ServiceProxy('/get_offline_recognise_result_srv',
                                       Get_Offline_Result_srv)
            # 参数：是否开始识别 / 置信度阈值 / 单次录音时长（秒）
            resp = proxy(1, 18, 15)
            if resp.result == 'ok' or resp.text:
                return TriggerResponse(success=True, message=_u8(resp.text or ''))
            return TriggerResponse(success=False, message=_u8(resp.fail_reason or u'识别失败'))
        except Exception as exc:
            return TriggerResponse(success=False, message=_u8(u'识别调用失败：%s' % repr(exc)))

    def on_set_awake_word(self, req):
        word = _to_unicode(getattr(req, 'message', '') or '').strip()
        if not word:
            return TriggerResponse(success=False, message=_u8(u'请提供新的唤醒词'))
        try:
            from xf_mic_asr_offline.srv import Set_Awake_Word_srv
            rospy.wait_for_service('/set_awake_word_srv', timeout=3.0)
            proxy = rospy.ServiceProxy('/set_awake_word_srv', Set_Awake_Word_srv)
            resp = proxy(word)
            return TriggerResponse(success=(resp.result == 'ok'),
                                   message=_u8(resp.result + ' ' + (resp.fail_reason or '')))
        except Exception as exc:
            return TriggerResponse(success=False, message=_u8(u'设置唤醒词失败：%s' % repr(exc)))

    def on_led(self, req):
        raw = _to_unicode(getattr(req, 'message', '') or '').strip() or '99'
        try:
            led_id = int(raw)
        except ValueError:
            return TriggerResponse(success=False, message=_u8(u'LED id 必须是整数，99 = 关灯'))
        try:
            from xf_mic_asr_offline.srv import Set_Led_On_srv
            rospy.wait_for_service('/set_target_led_on_srv', timeout=3.0)
            proxy = rospy.ServiceProxy('/set_target_led_on_srv', Set_Led_On_srv)
            resp = proxy(led_id)
            return TriggerResponse(success=(resp.result == 'ok'),
                                   message=_u8(resp.result + ' ' + (resp.fail_reason or '')))
        except Exception as exc:
            return TriggerResponse(success=False, message=_u8(u'控制 LED 失败：%s' % repr(exc)))

    def on_stop(self, _req):
        with self.queue_cv:
            self.queue = []
            self.stop_flag = True
            self.queue_cv.notify()
        proc = self.current_proc
        if proc is not None:
            try:
                proc.terminate()
            except Exception:
                pass
        return TriggerResponse(success=True, message=_u8(u'已停止播报并清空队列'))

    # ------------------------------------------------------------ 工具
    def _set_status(self, text):
        self.pub_status.publish(String(data=_u8(text)))
        rospy.loginfo('[voice_bridge] %s', _u8(text))


if __name__ == '__main__':
    setup_stdout()
    fix_ros_logging()
    rospy.init_node('voice_bridge')
    try:
        VoiceBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
