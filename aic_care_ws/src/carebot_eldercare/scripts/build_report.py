#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
build_report.py — 智慧养老护理赛题：从任务事件日志生成技术报告数据

用途
----
`mission_manager` 会把每一轮任务的完整过程写成 JSON Lines 事件日志
（默认 `/tmp/carebot_eldercare/mission_events.jsonl`）。本脚本把这些原始日志
汇总成技术报告里可以直接引用的**量化数据与表格**。

为什么需要它
------------
技术报告占 20 分（全部分值的 21%），而评委最容易给高分的是**有量化数据**的报告。
「识别效果良好」拿不到分；「在 1.5m 距离、200lux 照度下，20 次运行中
二维码识别成功 19 次，平均耗时 0.8s」才能拿分。
本脚本就是把这些数字自动算出来的工具。

用法
----
    # 汇总最新一轮
    rosrun carebot_eldercare build_report.py /tmp/carebot_eldercare/mission_events.jsonl

    # 汇总多轮（把历次日志都列上）
    rosrun carebot_eldercare build_report.py run1.jsonl run2.jsonl run3.jsonl

    # 输出 Markdown 表格，直接粘进报告
    rosrun carebot_eldercare build_report.py *.jsonl --markdown

    # 输出 JSON（供进一步分析/画图）
    rosrun carebot_eldercare build_report.py *.jsonl --json /tmp/stats.json
"""

import argparse
import json
import os
import sys
from collections import defaultdict, OrderedDict

try:
    import io
except ImportError:
    io = None

try:
    unicode  # noqa: F821  (Python 2)
except NameError:
    unicode = str  # noqa: A001


def open_text(path, mode='r'):
    """以 UTF-8 打开文本文件（显式指定编码，避免 Py2/Windows 下默认 GBK 报错）。"""
    if io is not None:
        return io.open(path, mode, encoding='utf-8')
    return open(path, mode)


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

# 关键事件 -> 中文标签（用于生成报告表格）
EVENT_LABELS = {
    'mission_begin':        u'任务开始',
    'mission_end':          u'任务结束',
    'nav_reached':          u'导航到达',
    'nav_failed':           u'导航失败',
    'qr_detected':          u'二维码识别成功',
    'qr_timeout':           u'二维码识别超时',
    'room_order_final':     u'最终巡视顺序',
    'fallback_default_order': u'降级为默认顺序',
    'face_verify':          u'人脸核验',
    'control_ok':           u'设备控制成功',
    'control_failed':       u'设备控制失败',
    'smoke_read':           u'烟雾读取',
    'leak_read':            u'漏水检测',
    'env_announced':        u'环境数据播报',
    'returned_to_start':    u'返回起点成功',
    'return_failed':        u'返回起点失败',
}

# 赛题评分表：项目名 -> (分值, 判定该得分的事件)
SCORE_ITEMS = [
    (u'机器人开机启动自主导航', 5, ['check_move_base']),
    (u'客厅光照强度检测',       5, ['env_announced']),
    (u'客厅温湿度检测',         5, ['env_announced']),
    (u'卧室电灯/窗帘控制',      10, ['control_ok']),
    (u'厨房烟雾检测',           5, ['smoke_read']),
    (u'厨房风扇控制',           10, ['control_ok']),
    (u'书房漏水检测',           5, ['leak_read']),
    (u'地图构建与准确导航',     5, ['nav_reached']),
    (u'语音交互',               10, ['say']),
    (u'返回起点',               5, ['returned_to_start']),
    (u'二维码识别',             15, ['qr_detected']),
]


def load_events(paths):
    """读取一个或多个 JSONL 文件，返回 (records, per_run)。"""
    per_run = []
    for path in paths:
        if not os.path.isfile(path):
            sys.stderr.write('跳过不存在的文件：%s\n' % path)
            continue
        recs = []
        with open_text(path, 'r') as fp:
            for line in fp:
                line = line.strip()
                if not line:
                    continue
                try:
                    recs.append(json.loads(line))
                except ValueError:
                    continue
        if recs:
            per_run.append({'path': path, 'records': recs})
    return per_run


def analyse_run(records):
    """分析单轮任务，返回统计字典。"""
    stats = {
        'duration_sec': None,
        'states': [],
        'nav_ok': 0,
        'nav_fail': 0,
        'nav_details': [],
        'qr_ok': False,
        'qr_payload': None,
        'room_order': None,
        'fallback_order': False,
        'face_passed': None,
        'control_ok': 0,
        'control_fail': 0,
        'control_details': [],
        'smoke_text': None,
        'leak_text': None,
        'leak_abnormal': None,
        'env_parts': [],
        'returned': False,
        'says': 0,
        'errors': 0,
        'warnings': 0,
        'end_state': None,
        'device_actions': [],
    }

    t0 = None
    for rec in records:
        ev = rec.get('event')
        lvl = rec.get('level')
        det = rec.get('detail')

        if lvl == 'error':
            stats['errors'] += 1
        elif lvl == 'warn':
            stats['warnings'] += 1

        if ev == 'mission_begin':
            t0 = rec.get('t', 0.0)
        elif ev == 'state_enter':
            stats['states'].append(rec.get('state'))

        elif ev == 'nav_reached':
            stats['nav_ok'] += 1
            if isinstance(det, dict):
                stats['nav_details'].append({
                    'waypoint': det.get('waypoint'),
                    'attempts': det.get('attempt'),
                })
        elif ev == 'nav_failed':
            stats['nav_fail'] += 1
            if isinstance(det, dict):
                stats['nav_details'].append({
                    'waypoint': det.get('waypoint'),
                    'result': 'failed',
                    'status': det.get('status'),
                    'attempts': det.get('attempt'),
                })

        elif ev == 'qr_detected':
            stats['qr_ok'] = True
            stats['qr_payload'] = det
        elif ev == 'room_order_final':
            stats['room_order'] = det
        elif ev == 'fallback_default_order':
            stats['fallback_order'] = True

        elif ev == 'face_verify' and isinstance(det, dict):
            stats['face_passed'] = bool(det.get('passed'))

        elif ev == 'control_ok' and isinstance(det, dict):
            stats['control_ok'] += 1
            stats['device_actions'].append('%s→%s' % (det.get('device'), det.get('action')))
        elif ev == 'control_failed' and isinstance(det, dict):
            stats['control_fail'] += 1
            stats['device_actions'].append('%s→%s(失败)' % (det.get('device'), det.get('action')))

        elif ev == 'smoke_read' and isinstance(det, dict):
            stats['smoke_text'] = det.get('text')
        elif ev == 'leak_read' and isinstance(det, dict):
            stats['leak_text'] = det.get('text')
            stats['leak_abnormal'] = det.get('abnormal')
        elif ev == 'env_announced' and isinstance(det, list):
            stats['env_parts'] = det

        elif ev == 'returned_to_start':
            stats['returned'] = True

        elif ev == 'say':
            stats['says'] += 1

        elif ev == 'mission_end' and isinstance(det, dict):
            stats['duration_sec'] = det.get('duration_sec')
            stats['end_state'] = 'DONE'

    if stats['duration_sec'] is None and t0 is not None:
        stats['duration_sec'] = round(records[-1].get('t', 0.0) - t0, 2) \
            if records else None

    # 状态流转是否完整
    seq = stats['states']
    stats['reached_done'] = 'DONE' in seq
    stats['reached_aborted'] = 'ABORTED' in seq
    return stats


def summarise(runs):
    """跨多轮汇总，算成功率。"""
    n = len(runs)
    if n == 0:
        return None

    def rate(pred):
        return round(100.0 * sum(1 for r in runs if pred(r)) / n, 1)

    durations = [r['stats']['duration_sec'] for r in runs
                 if r['stats']['duration_sec'] is not None]
    nav_ok = sum(r['stats']['nav_ok'] for r in runs)
    nav_fail = sum(r['stats']['nav_fail'] for r in runs)

    return OrderedDict([
        ('runs', n),
        (u'任务完整跑通率(%)', rate(lambda r: r['stats']['reached_done'])),
        (u'二维码识别成功率(%)', rate(lambda r: r['stats']['qr_ok'])),
        (u'人脸核验通过率(%)', rate(lambda r: r['stats']['face_passed'] is True)),
        (u'返回起点成功率(%)', rate(lambda r: r['stats']['returned'])),
        (u'平均用时(秒)', round(sum(durations) / len(durations), 1) if durations else None),
        (u'最短用时(秒)', min(durations) if durations else None),
        (u'最长用时(秒)', max(durations) if durations else None),
        (u'导航成功率(%)', round(100.0 * nav_ok / (nav_ok + nav_fail), 1)
            if (nav_ok + nav_fail) else None),
        (u'导航总次数', nav_ok + nav_fail),
        (u'设备控制成功次数', sum(r['stats']['control_ok'] for r in runs)),
        (u'设备控制失败次数', sum(r['stats']['control_fail'] for r in runs)),
        (u'语音播报总条数', sum(r['stats']['says'] for r in runs)),
        (u'错误事件总数', sum(r['stats']['errors'] for r in runs)),
        (u'警告事件总数', sum(r['stats']['warnings'] for r in runs)),
        (u'降级为默认顺序次数', sum(1 for r in runs if r['stats']['fallback_order'])),
    ])


def markdown_report(runs, summary):
    """生成可直接粘进技术报告的 Markdown。"""
    lines = []
    lines.append(u'## 任务运行统计（自动生成）\n')
    lines.append(u'> 数据来源：`mission_manager` 事件日志，共 %d 轮运行。\n' % len(runs))

    lines.append(u'\n### 总体指标\n')
    lines.append(u'| 指标 | 数值 |')
    lines.append(u'|---|---|')
    for k, v in summary.items():
        lines.append(u'| %s | %s |' % (k, u'—' if v is None else v))

    lines.append(u'\n### 逐轮明细\n')
    lines.append(u'| # | 日志文件 | 用时(秒) | 二维码 | 巡视顺序 | 人脸 | 导航(成功/失败) | 设备控制(成功/失败) | 结果 |')
    lines.append(u'|---|---|---|---|---|---|---|---|---|')
    for i, r in enumerate(runs, 1):
        s = r['stats']
        order = u''.join(str(x) for x in (s['room_order'] or [])) or u'—'
        lines.append(u'| %d | %s | %s | %s | %s | %s | %d/%d | %d/%d | %s |' % (
            i,
            os.path.basename(r['path']),
            u'—' if s['duration_sec'] is None else s['duration_sec'],
            u'成功' if s['qr_ok'] else u'失败',
            order,
            u'—' if s['face_passed'] is None else (u'通过' if s['face_passed'] else u'未通过'),
            s['nav_ok'], s['nav_fail'],
            s['control_ok'], s['control_fail'],
            u'完成' if s['reached_done'] else (u'中止' if s['reached_aborted'] else u'未完成'),
        ))

    lines.append(u'\n### 房间任务与实测数据\n')
    lines.append(u'| # | 环境数据 | 烟雾 | 漏水 | 设备动作序列 |')
    lines.append(u'|---|---|---|---|---|')
    for i, r in enumerate(runs, 1):
        s = r['stats']
        lines.append(u'| %d | %s | %s | %s | %s |' % (
            i,
            u'；'.join(s['env_parts']) or u'—',
            s['smoke_text'] or u'—',
            s['leak_text'] or u'—',
            u', '.join(s['device_actions']) or u'—',
        ))

    lines.append(u'\n### 失败模式分析\n')
    fail_items = []
    for r in runs:
        for nd in r['stats']['nav_details']:
            if nd.get('result') == 'failed':
                fail_items.append(u'- `%s` 导航到 **%s** 失败（状态 %s，尝试 %s 次）'
                                  % (os.path.basename(r['path']), nd.get('waypoint'),
                                     nd.get('status'), nd.get('attempts')))
    if fail_items:
        lines.extend(fail_items)
    else:
        lines.append(u'本轮统计中未出现导航失败。')

    return u'\n'.join(lines)


def main():
    setup_stdout()
    parser = argparse.ArgumentParser(
        description='从任务事件日志生成技术报告数据')
    parser.add_argument('logs', nargs='+', help='mission_events.jsonl 文件（可多个）')
    parser.add_argument('--markdown', action='store_true', help='输出 Markdown 表格')
    parser.add_argument('--json', default='', help='把统计写入 JSON 文件')
    parser.add_argument('--out', default='', help='把报告写入文件（默认打印到终端）')
    args = parser.parse_args()

    per_run = load_events(args.logs)
    if not per_run:
        sys.stderr.write('没有读到任何事件记录。\n')
        return 2

    runs = []
    for item in per_run:
        runs.append({'path': item['path'], 'stats': analyse_run(item['records'])})

    summary = summarise(runs)

    if args.markdown:
        text = markdown_report(runs, summary)
    else:
        out = [u'任务运行统计（共 %d 轮）' % len(runs), u'=' * 60]
        for k, v in summary.items():
            out.append(u'%-24s %s' % (k, u'—' if v is None else v))
        out.append(u'')
        out.append(u'逐轮明细：')
        for i, r in enumerate(runs, 1):
            s = r['stats']
            out.append(u'  第 %d 轮  %s' % (i, os.path.basename(r['path'])))
            out.append(u'    用时      : %s 秒' % (s['duration_sec'] if s['duration_sec'] is not None else u'—'))
            out.append(u'    二维码    : %s  载荷=%s' % (u'成功' if s['qr_ok'] else u'失败', s['qr_payload']))
            out.append(u'    巡视顺序  : %s' % (s['room_order'] or u'—'))
            out.append(u'    人脸核验  : %s' % (u'—' if s['face_passed'] is None else (u'通过' if s['face_passed'] else u'未通过')))
            out.append(u'    导航      : 成功 %d / 失败 %d' % (s['nav_ok'], s['nav_fail']))
            out.append(u'    设备控制  : 成功 %d / 失败 %d' % (s['control_ok'], s['control_fail']))
            out.append(u'    环境播报  : %s' % (u'；'.join(s['env_parts']) or u'—'))
            out.append(u'    烟雾/漏水 : %s / %s' % (s['smoke_text'] or u'—', s['leak_text'] or u'—'))
            out.append(u'    语音播报  : %d 条' % s['says'])
            out.append(u'    结果      : %s' % (u'完成' if s['reached_done'] else (u'中止' if s['reached_aborted'] else u'未完成')))
        text = u'\n'.join(out)

    if args.out:
        with open_text(args.out, 'w') as fp:
            fp.write(text)
        print('报告已写入：%s' % args.out)
    else:
        print(text)

    if args.json:
        payload = {'summary': summary,
                   'runs': [{'path': r['path'], 'stats': r['stats']} for r in runs]}
        with open_text(args.json, 'w') as fp:
            json.dump(payload, fp, ensure_ascii=False, indent=2)
        print('\nJSON 统计已写入：%s' % args.json)

    return 0


if __name__ == '__main__':
    sys.exit(main())
