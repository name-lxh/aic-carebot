# -*- coding: utf-8 -*-
"""voice_dialog.py —— 到达 A 点后的五轮语音交互（按现场设计的动线）

## 流程（2026-10-23 现场改版：3 轮 -> 5 轮，第 2 轮起必须先唤醒）

    第 1 轮（机器人主动问，不用唤醒）
        车："主人，有什么我能帮你的"
        问完停 1 s 再开麦，收音 15 s
        你："导航到厨房"
        车：导航过去 -> "已经到达厨房"

    第 2~5 轮（必须先唤醒）
        车：（静默等唤醒词，最多 20 s）
        你："老张同学"   -> 车："我在"
        车：停 1 s 再开麦，收音 10 s
        你：任意指令（不喊唤醒词说的指令会被忽略，并提示要先喊）

## 硬件指令与回执（都会回读状态再回答）

    我有点热 / 打开风扇  -> fan_kitchen on   -> "已经打开风扇了"
    关闭风扇            -> fan_kitchen off  -> "风扇已经关掉了"
    开灯 / 关灯         -> light_bedroom    -> "已经打开灯了 / 已经关灯了"
    打开窗帘 / 关闭窗帘  -> curtain_bedroom  -> "已经打开窗帘了 / 已经关闭窗帘了"
    温度查询            -> /iot/refresh     -> "当前室内温度 21.9 摄氏度"

## 自由控制（2026-10-22 现场要求）

五轮**都可以执行任意任务**，不按轮次区分；任何一轮说"没什么事"都立即结束。
唤醒词"老张同学"从第 2 轮起是**必须**的：先说唤醒词、听到"我在"，再说指令。

## 关键时序（现场要求）

    播报结束后 **再等 1 秒** 才开麦，避免把自己的尾音当回答；
    第 1 轮收音 **15 秒**，第 2~5 轮各 **10 秒**；共 **5 轮**。
"""
from __future__ import print_function

import codecs
import math
import Queue
import sys
import time

import rospy
from std_msgs.msg import String

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

sys.path.insert(0, '/home/ub/Hajimi/LXH/_tools')

ASK_TEXT = u'主人，有什么我能帮你的'
ACK_WAKE = u'我在'
BYE_TEXT = u'好的，我随时都在'
NO_REPLY = 15.0          # 收音窗口（第 2~5 轮；10 s 现场偏紧，2026-10-23 调到 15 s）
NO_REPLY_FIRST = 15.0    # ★ 第 1 轮放宽到 15 s（现场：听到提问后反应+开口需要时间）
WAKE_TIMEOUT = 20.0      # 第 2~5 轮等唤醒词的最长时间
MAX_ROUNDS = 5           # 总轮数（2026-10-23：3 -> 5；第 2 轮起必须先喊"老张同学"）

# ★ 2026-10-23：一次唤醒的"会话模型"
#   VOICE_SESSION=True 时不限轮次：每条有效指令把窗口续到 WINDOW_SESSION 秒，
#   唤醒词只续期、不占轮次；静默 / 说"没什么事" / 超时 结束会话。
#   设 False 可整体退回原来的 5 轮模型。
VOICE_SESSION = True
WINDOW_FIRST = 15.0      # 第 1 轮（车先问）的静默窗口
WINDOW_SESSION = 20.0    # 会话内每条指令后的窗口
SESSION_MAX = 180.0      # 会话总时长上限（防卡死）
GUARD_XY = 0.08          # 静止守卫：位移超 8 cm 认为车被动过
GUARD_YAW = 0.18         # 静止守卫：朝向变化超 0.18 rad
HW_WORDS = {
    'fan_on': u'已经打开风扇了',
    'fan_off': u'风扇已经关掉了',
    'light_on': u'已经打开灯了',
    'light_off': u'已经关灯了',
    'curtain_open': u'已经打开窗帘了',
    'curtain_close': u'已经关闭窗帘了',
}
CN = {'C': u'厨房', 'D': u'卧室', 'B': u'书房', 'living': u'客厅'}


class Dialog(object):
    def __init__(self, say_fn, temp_fn, nav_fn, hw_fn=None):
        self.say = say_fn
        self.get_temp = temp_fn
        self.nav_room = nav_fn
        self.hw = hw_fn
        self.q = Queue.Queue()
        self.logs = []
        self._tf = None
        rospy.Subscriber('/voice_words', String, self._cb, queue_size=10)

    # ------------------------------------------------------------ 静止守卫
    def _pose(self):
        """返回 (x, y, yaw)；拿不到就 None（不阻塞对话）。"""
        try:
            if self._tf is None:
                import tf
                self._tf = tf.TransformListener()
            import tf as _tf
            self._tf.waitForTransform('map', 'base_footprint',
                                      rospy.Time(0), rospy.Duration(0.3))
            trans, rot = self._tf.lookupTransform('map', 'base_footprint',
                                                  rospy.Time(0))
            return (trans[0], trans[1],
                    _tf.transformations.euler_from_quaternion(rot)[2])
        except Exception:
            return None

    def _guard_broken(self, ref):
        if not ref:
            return False
        cur = self._pose()
        if not cur:
            return False
        if math.hypot(cur[0] - ref[0], cur[1] - ref[1]) > GUARD_XY:
            print(u'  [守卫] 车被移动 %.2f m' % math.hypot(cur[0] - ref[0], cur[1] - ref[1]))
            return True
        dyaw = abs(math.atan2(math.sin(cur[2] - ref[2]), math.cos(cur[2] - ref[2])))
        if dyaw > GUARD_YAW:
            print(u'  [守卫] 车被转动 %.2f rad' % dyaw)
            return True
        return False

    # ------------------------------------------------------------ 基础
    def _cb(self, msg):
        try:
            text = msg.data.decode('utf-8') if isinstance(msg.data, str) else msg.data
        except Exception:
            text = u''
        text = text.strip()
        if text:
            self.q.put(text)

    def _drain(self):
        n = 0
        while True:
            try:
                self.q.get_nowait()
                n += 1
            except Queue.Empty:
                return n

    def _say(self, text, extra=0.6):
        """播报；等它说完 + extra 秒，再清空缓冲。

        extra 的取法：
          * 提问（"主人，有什么我能帮你的"）用 1.0 —— 现场要求"问完 1 s 再开麦"
          * 短回复（"我在"）用 0.4 —— 等太久会把用户紧接着说的话当成回声丢掉
        """
        self.say(text)
        self.logs.append(text)
        wait = max(1.2, len(text) * 0.28) + extra
        time.sleep(wait)
        d = self._drain()
        if d:
            print(u'     (丢弃播报期间的 %d 条回声)' % d)

    def _listen(self, window):
        """在 window 秒内等一句识别结果。"""
        t0 = time.time()
        while time.time() - t0 < window and not rospy.is_shutdown():
            try:
                text = self.q.get(timeout=0.2)
            except Queue.Empty:
                continue
            print(u'  [听到] %s' % text)
            return text
        return None

    # ------------------------------------------------------------ 意图
    def _match(self, text):
        from intent_match import match
        return match(text)

    def _do_hw(self, intent):
        if not self.hw:
            self._say(u'暂时不能控制设备')
            return
        ok, msg = self.hw(intent)
        if ok:
            self._say(HW_WORDS.get(intent, u'已经执行'))
        else:
            self._say(u'执行失败，请再试一次')
            print(u'     (硬件 %s 失败: %r)' % (intent, msg))

    # ------------------------------------------------------------ 主流程
    def run(self):
        if VOICE_SESSION:
            return self._run_session()
        self._drain()
        rnd = 0
        while rnd < MAX_ROUNDS and not rospy.is_shutdown():
            rnd += 1
            print(u'')
            print(u'  ═══ 第 %d/%d 轮 ═══' % (rnd, MAX_ROUNDS))

            if rnd == 1:
                self._say(ASK_TEXT, extra=1.0)
            else:
                print(u'  （第 %d 轮：请先说"老张同学"，最多等 %.0f s）'
                      % (rnd, WAKE_TIMEOUT))
                got = None
                t0 = time.time()
                while time.time() - t0 < WAKE_TIMEOUT and not rospy.is_shutdown():
                    left = WAKE_TIMEOUT - (time.time() - t0)
                    text = self._listen(min(2.0, max(0.2, left)))
                    if not text:
                        continue
                    r = self._match(text)
                    if r and r[0] == 'wake':
                        got = text
                        break
                    # ★ 2026-10-23：第 2 轮起必须先喊唤醒词，直接说指令不放行。
                    #   只在日志里提示，**不出声**（现场要求不要额外播报）。
                    print(u'  [提示] 第 %d 轮要先喊"老张同学"，已忽略：%s'
                          % (rnd, text))
                if not got:
                    self._say(u'那我先忙了')
                    print(u'  [结束] 没等到唤醒词')
                    return False
                self._say(ACK_WAKE, extra=0.4)

            win = NO_REPLY_FIRST if rnd == 1 else NO_REPLY
            intent = hit = score = None
            # ★ 2026-10-23：本轮改成**按时间**听，不按“听几次”。
            #   原来最多听 3 次，用户连喊几声“老张同学”就把 3 次用光了，
            #   随后的指令整句没人接（实测就是这样漏掉“关闭厨房风扇 9 分”）。
            #   现在：唤醒词回“我在”并**重置窗口**，只有真的等满 15 s 没话才收尾。
            deadline = time.time() + win
            heard_any = False
            while time.time() < deadline and not rospy.is_shutdown():
                left = max(0.2, min(2.0, deadline - time.time()))
                text = self._listen(left)
                if not text:
                    continue
                heard_any = True
                r = self._match(text)
                if not r:
                    self._say(u'没听清，请再说一次')
                    continue
                if r[0] == 'wake':
                    self._say(ACK_WAKE, extra=0.4)
                    print(u'  [提示] 唤醒词不占轮次，继续听指令')
                    deadline = time.time() + win
                    continue
                intent, hit, score = r
                print(u'  [意图] %s（命中"%s" %.2f）' % (intent, hit, score))
                break
            if intent is None:
                self._say(u'那我先忙了')
                print(u'  [结束] %.0f s 内没有可用指令（听到过唤醒词：%s）'
                      % (win, u'是' if heard_any else u'否'))
                return False

            if self._do_intent(intent) == 'stop':
                self._say(BYE_TEXT)
                print(u'  [结束] 用户说没事了')
                return True
            continue

        self._say(BYE_TEXT)
        print(u'  [结束] 已满 %d 轮' % MAX_ROUNDS)
        return True

    # ------------------------------------------------------- 会话模型
    def _do_intent(self, intent):
        """执行一条已匹配的意图。返回 'stop' 表示结束会话。"""
        if intent == 'stop':
            return 'stop'
        if intent == 'temp':
            try:
                self._say(self.get_temp())
            except Exception as exc:
                print(u'  (读温度失败: %r)' % exc)
                self._say(u'暂时读不到温度')
            return 'done'
        if intent.startswith('goto:'):
            key = intent.split(':', 1)[1]
            cn = CN.get(key, key)
            self._say(u'好的，正在去%s' % cn)
            ok = self.nav_room(key)
            self._say(u'已经到达%s' % cn if ok else u'没能到达%s' % cn)
            return 'done'
        if intent == 'hot':
            self._do_hw('fan_on')
            return 'done'
        if intent in HW_WORDS:
            self._do_hw(intent)
            return 'done'
        self._say(u'没听清，请再说一次')
        return 'done'

    def _run_session(self):
        """一次唤醒的会话：每条指令把窗口续到 WINDOW_SESSION 秒。"""
        self._drain()
        print(u'  ═══ 语音会话（一次唤醒，窗口 %.0f s）═══' % WINDOW_SESSION)
        self._say(ASK_TEXT, extra=1.0)
        t_start = time.time()
        deadline = t_start + WINDOW_FIRST
        ref = self._pose()
        while not rospy.is_shutdown():
            now = time.time()
            if now - t_start > SESSION_MAX:
                self._say(u'时间到，我先去忙了')
                print(u'  [结束] 会话超过 %.0f s' % SESSION_MAX)
                return True
            if now >= deadline:
                self._say(BYE_TEXT)
                print(u'  [结束] 静默 %.0f s 收尾' % WINDOW_SESSION)
                return True
            if self._guard_broken(ref):
                self._say(BYE_TEXT)
                print(u'  [结束] 车被移动，会话中止')
                return True
            text = self._listen(max(0.2, min(2.0, deadline - time.time())))
            if not text:
                continue
            r = self._match(text)
            if not r:
                self._say(u'没听清，请再说一次')      # 不续期
                continue
            if r[0] == 'wake':
                self._say(ACK_WAKE, extra=0.4)
                deadline = time.time() + WINDOW_SESSION
                print(u'  [会话] 唤醒词：窗口续 %.0f s' % WINDOW_SESSION)
                continue
            intent, hit, score = r
            print(u'  [意图] %s（命中"%s" %.2f）' % (intent, hit, score))
            if self._do_intent(intent) == 'stop':
                self._say(BYE_TEXT)
                print(u'  [结束] 用户说没事了')
                return True
            ref = self._pose()
            deadline = time.time() + WINDOW_SESSION
            print(u'  [会话] 指令完成：窗口续 %.0f s' % WINDOW_SESSION)
        return True


# ---------------------------------------------------------------- 独立测试
def main():
    import m36_mission_v5 as m

    rospy.init_node('voice_dialog', anonymous=True)
    say_pub = rospy.Publisher('/voice/say', String, queue_size=5)
    rospy.sleep(0.8)

    def say(text):
        print(u'  [播报] %s' % text)
        say_pub.publish(String(data=text.encode('utf-8')))

    dev = m.RosDevices(fake_iot=False)

    def get_temp():
        data = dev.iot_refresh()
        env = (data or {}).get('env') or {}
        t = env.get('temperature') or {}
        if t.get('value') is None:
            return u'暂时读不到温度'
        return u'当前室内温度 %.1f 摄氏度' % float(t['value'])

    nav = [None]

    def nav_room(key):
        if nav[0] is None:
            nav[0] = m.Nav(m.Guard(enabled=False))
        if key == 'living':
            st, _, _ = nav[0].goto_named('living_in')
        else:
            x, y, _ = m.WP[key]
            st, _, _ = nav[0].goto(u'语音导航-%s' % key, x, y, None)
        return st in ('SUCCEEDED', 'DRY')

    def hw(intent):
        mp = {'fan_on': ('fan_kitchen', 'on'), 'fan_off': ('fan_kitchen', 'off'),
              'light_on': ('light_bedroom', 'on'), 'light_off': ('light_bedroom', 'off'),
              'curtain_open': ('curtain_bedroom', 'open'),
              'curtain_close': ('curtain_bedroom', 'close')}
        d, a = mp[intent]
        return dev.iot_control(d, a)

    d = Dialog(say, get_temp, nav_room, hw)
    return 0 if d.run() else 1


if __name__ == '__main__':
    sys.exit(main())
