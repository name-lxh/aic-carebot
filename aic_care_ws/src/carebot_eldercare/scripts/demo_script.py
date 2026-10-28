#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
demo_script.py — 智慧养老护理赛题：演示脚本与录制辅助

这个脚本解决两个很实际的交付问题
--------------------------------
1. **线上赛的"一镜到底"到底该说什么** —— 赛题要求语音讲解，但没说讲什么。
   本脚本提供逐句解说词，并把它**真正送进 TTS 播报**，让视频里的每一句话都真实来自机器人。
2. **录制后要能对上时间轴** —— 生成一份带时间戳的脚本文档，
   剪辑/复盘时能立刻知道"第 37 秒发生了什么"。

三种用法
--------
    # 打印脚本（不连机器人也能用，用于队员背稿）
    python demo_script.py --print

    # 真机演示：按脚本顺序播报解说词（用 /voice/announce，等待播完）
    python demo_script.py --play

    # 加 --markers 会在每句播报前写一条时间戳标记，录制后用于对齐
    python demo_script.py --play --markers --out ~/demo_timeline.md

    # 只导出脚本为 Markdown（交给配音/主持同学）
    python demo_script.py --print --out ~/演示解说词.md

设计原则
--------
* 解说词与状态机自身的自动播报**错开**：状态机负责"操作播报"（已到达厨房、
  已打开风扇），本脚本负责"讲解播报"（现在它要做什么、为什么这么做）。
* 每句都标注**对应评分项**，方便复盘时确认有没有漏讲。
"""

import argparse
import io
import json
import os
import subprocess
import sys
import time

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


def open_text(path, mode='r'):
    return io.open(path, mode, encoding='utf-8')


# ---------------------------------------------------------------------------
# 演示脚本：与任务状态机的阶段一一对应
#   ('阶段名', '解说词', '对应评分项')
# ---------------------------------------------------------------------------
SCRIPT = [
    (u'开场',
     u'您好，我是智慧养老护理机器人。我搭载了激光雷达、深度相机和六麦克风阵列，'
     u'能够自主建图导航，并通过智能家居网关与家里的电器联动。现在我开始执行居家巡视任务。',
     u'机器人开机（5 分）/ 语音交互（10 分）'),

    (u'出发',
     u'我已经启动自主导航，接下来全程不需要任何人工干预。',
     u'机器人开机（5 分）— 强调"禁止人为控制"'),

    (u'寻找二维码',
     u'房间里贴着一张二维码，上面写着我需要巡视三个房间的顺序。'
     u'我正在前往二维码张贴区域。',
     u'二维码识别（15 分）'),

    (u'识别二维码',
     u'我看到二维码了，正在读取并解析巡视顺序。',
     u'二维码识别（15 分）'),

    (u'确认顺序',
     u'读取完成，我将按照这个顺序依次巡视各房间。',
     u'二维码识别（15 分）'),

    (u'客厅环境',
     u'现在检测客厅的环境。房间的光照强度、温度和湿度数据我已经采集完成，'
     u'这些数据可以判断老人的居住环境是否舒适。',
     u'光照强度（5 分）+ 温湿度（5 分）'),

    (u'前往核验点',
     u'在进入其他房间之前，我需要先在 A 点确认您的身份。'
     u'这是为了保障居家安全，只有识别到主人才会继续执行。',
     u'规则门槛：人脸识别后方可进入房间'),

    (u'人脸核验',
     u'正在采集人脸特征并与已登记的主人信息比对。',
     u'规则门槛：人脸识别'),

    (u'卧室任务',
     u'第一位是卧室。老人休息的区域需要合适的光线，'
     u'我现在打开卧室的电灯和窗帘。',
     u'卧室电灯/窗帘控制（10 分）'),

    (u'厨房任务',
     u'接下来是厨房。厨房是燃气和烟雾风险最高的地方，'
     u'我先读取烟雾报警器的数据，然后打开风扇保持通风。',
     u'厨房烟雾检测（5 分）+ 风扇控制（10 分）'),

    (u'书房任务',
     u'最后是书房。漏水如果不及时发现会造成很大损失，'
     u'我现在执行漏水检测。',
     u'书房漏水检测（5 分）'),

    (u'返回起点',
     u'三个房间巡视完毕。现在返回起点。'
     u'注意起点位置每次都是随机的，我在任务开始时记录了出发时的位置。',
     u'机器人返回起点（5 分）'),

    (u'收尾',
     u'任务完成。整个过程中机器人自主完成了建图定位、二维码识别、'
     u'人脸核验、环境感知和家电控制，全程零人工干预。谢谢观看。',
     u'地图构建（5 分）/ 总体展示'),
]

# 创意功能演示脚本（可选，用于创意挑战 20 分）
CREATIVE_SCRIPT = [
    (u'创意-声源定位',
     u'除了基础任务，我还具备声源定位能力。当有人呼唤我时，'
     u'六个麦克风阵列会计算出声源方向，我会主动转向说话的人。',
     u'创意挑战（20 分）'),
    (u'创意-跌倒预警',
     u'我还会持续监测室内人员的姿态。如果检测到人体长时间静止或姿态异常，'
     u'我会主动询问是否需要帮助，并在无应答时通过智能家居网关推送告警。',
     u'创意挑战（20 分）'),
]


def call_service(name, args=None, timeout=60.0):
    cmd = ['rosservice', 'call', name]
    if args:
        cmd.append(args)
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out, _ = proc.communicate()
        text = out.decode('utf-8', 'replace') if isinstance(out, bytes) else out
        return ('success: True' in text or 'success: true' in text.lower()), text.strip()
    except Exception as exc:
        return False, unicode(exc)


def render_markdown(script, title=u'演示解说词'):
    lines = [u'# %s\n' % title]
    lines.append(u'> 本脚本由 `demo_script.py` 生成。每句解说词均通过机器人'
                 u'语音合成实时播报，对应视频中的真实语音。\n')
    lines.append(u'| # | 阶段 | 解说词 | 对应评分项 |')
    lines.append(u'|---|---|---|---|')
    for i, (stage, text, score) in enumerate(script, 1):
        lines.append(u'| %d | %s | %s | %s |' % (i, stage, text, score))
    total = sum(len(t) for _s, t, _sc in script)
    lines.append(u'\n**统计**：共 %d 句，约 %d 字。'
                 u'按语速 5 字/秒估算，纯播报时长约 %.0f 秒。\n'
                 % (len(script), total, total / 5.0))
    return u'\n'.join(lines)


def main():
    setup_stdout()
    parser = argparse.ArgumentParser(
        description='智慧养老护理赛题 — 演示脚本与录制辅助')
    parser.add_argument('--print', dest='do_print', action='store_true',
                        help='打印脚本（不需要机器人）')
    parser.add_argument('--play', action='store_true',
                        help='按脚本顺序通过 TTS 播报（需要系统在运行）')
    parser.add_argument('--creative', action='store_true',
                        help='改用创意功能演示脚本')
    parser.add_argument('--markers', action='store_true',
                        help='播报前写时间戳标记（用于录制后对齐）')
    parser.add_argument('--gap', type=float, default=1.0,
                        help='两句之间的间隔（秒）')
    parser.add_argument('--out', default='', help='把脚本写入 Markdown 文件')
    args = parser.parse_args()

    script = CREATIVE_SCRIPT if args.creative else SCRIPT
    title = u'创意功能演示解说词' if args.creative else u'任务演示解说词'

    if not args.play:
        args.do_print = True

    # ---------------------------------------------------------------- 打印
    if args.do_print:
        print(u'\n' + u'=' * 72)
        print(u'  %s' % title)
        print(u'=' * 72)
        for i, (stage, text, score) in enumerate(script, 1):
            print(u'\n[%2d] %s' % (i, stage))
            print(u'     解说：%s' % text)
            print(u'     评分：%s' % score)
        total = sum(len(t) for _s, t, _sc in script)
        print(u'\n' + u'-' * 72)
        print(u'  共 %d 句 / 约 %d 字 / 纯播报约 %.0f 秒'
              % (len(script), total, total / 5.0))
        print(u'-' * 72)

    # ---------------------------------------------------------------- 输出
    if args.out:
        path = os.path.expanduser(args.out)
        d = os.path.dirname(path)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open_text(path, 'w') as fp:
            fp.write(render_markdown(script, title))
        print(u'\n解说词已写入：%s' % path)

    if not args.play:
        return 0

    # ---------------------------------------------------------------- 播报
    print(u'\n' + u'=' * 72)
    print(u'  开始演示播报（Ctrl+C 中止）')
    print(u'=' * 72)

    markers = []
    t0 = time.time()
    for i, (stage, text, score) in enumerate(script, 1):
        elapsed = time.time() - t0
        print(u'\n[%02d:%02d] %s' % (int(elapsed) // 60, int(elapsed) % 60, stage))
        print(u'         %s' % text[:60] + (u'...' if len(text) > 60 else u''))

        if args.markers:
            markers.append({'index': i, 'stage': stage,
                            't_sec': round(elapsed, 2),
                            'wall': time.strftime('%H:%M:%S'),
                            'text': text, 'score': score})

        ok, msg = call_service('/voice/announce', json.dumps(text), timeout=60.0)
        if not ok:
            # /voice/announce 需要 Trigger；换用话题发布兜底
            subprocess.call(['rostopic', 'pub', '-1', '/voice/say',
                             'std_msgs/String', text])
            print(u'         (服务不可用，已改用话题发布)')

        # 等播报结束（按字数估时长）
        wait = max(1.5, len(text) / 5.0) + args.gap
        time.sleep(min(wait, 30.0))

    total = time.time() - t0
    print(u'\n' + u'-' * 72)
    print(u'  演示播报结束，总时长 %.0f 秒' % total)
    print(u'-' * 72)

    if args.markers and markers:
        mpath = os.path.expanduser(args.out) if args.out else \
            os.path.expanduser('~/demo_timeline.json')
        if mpath.endswith('.md'):
            mpath = mpath[:-3] + u'_时间轴.json'
        with open_text(mpath, 'w') as fp:
            json.dump({'total_sec': round(total, 1), 'markers': markers},
                      fp, ensure_ascii=False, indent=2)
        print(u'  时间轴已写入：%s' % mpath)

    return 0


if __name__ == '__main__':
    sys.exit(main())
