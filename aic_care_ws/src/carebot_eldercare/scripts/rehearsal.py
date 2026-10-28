#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rehearsal.py — 智慧养老护理赛题：赛前彩排与数据采集一体机

用途
----
赛前的核心工作是**反复跑整轮任务并收集数据**，用于两件事：
  1. 发现并修掉不稳定的环节（成功率、平均用时）；
  2. 为技术报告产出**量化数据**（20 分，且必须用真实数据）。

手工做这件事很痛苦：要记时间、要抄结果、要重命名日志。本脚本把它自动化。

三种模式
--------
  --mode single     跑一轮，打印结果摘要
  --mode repeat     跑 N 轮，每轮一个独立日志文件，最后自动汇总统计
  --mode collect    不跑任务，只把已有的日志汇总成报告（等价于 build_report.py）

用法
----
    # 单轮彩排（需要 eldercare_full.launch 已在跑）
    rosrun carebot_eldercare rehearsal.py --mode single

    # 连跑 10 轮，日志存到 ~/eldercare_runs/
    rosrun carebot_eldercare rehearsal.py --mode repeat --runs 10

    # 汇总统计并生成 Markdown（可直接粘进技术报告）
    rosrun carebot_eldercare rehearsal.py --mode collect --runs-dir ~/eldercare_runs \\
        --markdown ~/技术报告_实测数据.md

    # 彩排时不想控实物（只验证逻辑与导航）
    rosrun carebot_eldercare rehearsal.py --mode repeat --runs 5 --dry-run

设计说明
--------
* 每轮之间强制间隔 `--gap` 秒，让机器人回到起点、让 HA 设备复位。
* 每轮开始前会**先复位实物状态**（灯/窗帘/风扇关掉），避免上一轮的效果影响下一轮。
* 日志文件名带时间戳与轮次，方便回溯。
"""

import argparse
import io
import json
import os
import shutil
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


def banner(text, char='='):
    print('\n' + char * 70)
    print('  ' + text)
    print(char * 70)


def call_service(name, args=None, timeout=180.0):
    """调用一个 std_srvs/Trigger 风格的服务，返回 (success, message)。

    用命令行 rosservice 而不是 rospy，好处是脚本可以在没有 rospy 环境时也能跑
    （--mode collect 就不需要 ROS）。
    """
    cmd = ['rosservice', 'call', name]
    if args:
        cmd.append(args)
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out, _ = proc.communicate()
        text = out.decode('utf-8', 'replace') if isinstance(out, bytes) else out
        ok = 'success: True' in text or 'success: true' in text.lower()
        return ok, text.strip()
    except Exception as exc:
        return False, '调用 %s 失败：%s' % (name, exc)


def publish_once(topic, msg_type, data):
    """发布一条 latched 消息（用于 /eldercare/start 这类触发）。"""
    cmd = ['rostopic', 'pub', '-1', topic, msg_type, data]
    try:
        subprocess.call(cmd)
        return True
    except Exception as exc:
        print('  发布 %s 失败：%s' % (topic, exc))
        return False


def check_ros_ready():
    """确认 ROS 与关键节点在线。返回 (ok, 说明)。"""
    try:
        proc = subprocess.Popen(['rostopic', 'list'], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        out, _ = proc.communicate()
        text = out.decode('utf-8', 'replace') if isinstance(out, bytes) else out
    except Exception as exc:
        return False, 'ROS 不可用：%s' % repr(exc)

    if 'rostopic: command not found' in text:
        return False, '未找到 rostopic，请先 source ROS 环境'

    topics = text.split()
    required = ['/move_base/goal', '/eldercare/mission_state']
    missing = [t for t in required if t not in topics]
    if missing:
        return False, '缺少关键话题：%s（请先 roslaunch carebot_eldercare eldercare_full.launch）' \
            % ', '.join(missing)
    return True, 'ROS 就绪'


def reset_devices(dry_run=False):
    """把实物设备复位到初始状态，保证每轮从同一条件开始。"""
    if dry_run:
        print('  [dry_run] 跳过设备复位')
        return
    resets = [
        ('light_bedroom', 'off', '关闭卧室电灯'),
        ('curtain_bedroom', 'close', '关闭卧室窗帘'),
        ('fan_kitchen', 'off', '关闭厨房风扇'),
    ]
    for device, action, label in resets:
        payload = json.dumps({'device': device, 'action': action})
        ok, _msg = call_service('/iot/control', payload, timeout=15.0)
        print('  复位 %-18s %s' % (label, 'OK' if ok else '失败/跳过'))


def run_one_round(run_index, log_dir, dry_run=False, timeout=300.0):
    """跑一轮任务，返回 (ok, duration, log_path, summary_message)。"""
    stamp = time.strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(log_dir, 'run%02d_%s.jsonl' % (run_index, stamp))

    print('\n  ── 第 %d 轮 ──────────────────────────────────────────' % run_index)
    print('  日志：%s' % log_path)

    # 让 mission_manager 把事件日志写到本轮的文件
    try:
        subprocess.call(['rosparam', 'set', '/mission_manager/event_log', log_path])
    except Exception:
        pass

    reset_devices(dry_run)
    time.sleep(1.0)

    t0 = time.time()
    # 先发一次 abort 清除可能残留的中止标志
    publish_once('/eldercare/abort', 'std_msgs/Bool', 'false')
    time.sleep(0.3)
    publish_once('/eldercare/start', 'std_msgs/Bool', 'true')

    print('  已触发任务，等待完成（最长 %.0f 秒）...' % timeout)

    # 轮询任务状态直到 DONE / ABORTED / 超时
    final_state = None
    last_print = 0.0
    while time.time() - t0 < timeout:
        try:
            proc = subprocess.Popen(
                ['rostopic', 'echo', '-n', '1', '/eldercare/mission_state'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            out, _ = proc.communicate()
            text = out.decode('utf-8', 'replace') if isinstance(out, bytes) else out
            for state in ('DONE', 'ABORTED'):
                if state in text:
                    final_state = state
                    break
        except Exception:
            pass

        if final_state:
            break

        elapsed = time.time() - t0
        if elapsed - last_print > 15.0:
            last_print = elapsed
            print('    ... 已运行 %.0f 秒' % elapsed)
        time.sleep(2.0)

    duration = time.time() - t0

    if final_state == 'DONE':
        ok = True
        print('  ✔ 完成，用时 %.1f 秒' % duration)
    elif final_state == 'ABORTED':
        ok = False
        print('  ✘ 中止，用时 %.1f 秒' % duration)
    else:
        ok = False
        print('  ✘ 超时（%.0f 秒未结束）' % timeout)

    # 落一份报告副本，便于后续汇总
    if os.path.isfile(log_path):
        rc, msg = call_service('/eldercare/get_report', timeout=30.0)
        if rc:
            try:
                # rosservice 输出里抠出 JSON
                start = msg.find('{')
                if start >= 0:
                    data = json.loads(msg[start:msg.rfind('}') + 1])
                    with open_text(log_path + '.summary.json', 'w') as fp:
                        json.dump({'round': run_index, 'duration_sec': round(duration, 1),
                                   'result': final_state, 'report': data},
                                  fp, ensure_ascii=False, indent=2)
            except Exception:
                pass

    return ok, duration, log_path, final_state


def mode_repeat(args):
    ok, msg = check_ros_ready()
    print('  ROS 检查：%s' % msg)
    if not ok and not args.force:
        print('\n  请先启动：roslaunch carebot_eldercare eldercare_full.launch')
        print('  或用 --force 跳过检查（不推荐）')
        return 2

    log_dir = os.path.expanduser(args.runs_dir)
    if not os.path.isdir(log_dir):
        os.makedirs(log_dir)
    print('  日志目录：%s' % log_dir)

    results = []
    for i in range(1, args.runs + 1):
        try:
            ok_i, dur, path, state = run_one_round(
                i, log_dir, dry_run=args.dry_run, timeout=args.timeout)
            results.append({'round': i, 'ok': ok_i, 'duration': round(dur, 1),
                            'state': state, 'log': path})
        except KeyboardInterrupt:
            print('\n  用户中断，提前结束。')
            break
        except Exception as exc:
            print('  第 %d 轮异常：%s' % (i, exc))
            results.append({'round': i, 'ok': False, 'duration': None,
                            'state': 'ERROR', 'log': None})

        if i < args.runs:
            print('  休息 %.0f 秒后开始下一轮...' % args.gap)
            time.sleep(args.gap)

    # 汇总
    banner('彩排结果汇总')
    n_ok = sum(1 for r in results if r['ok'])
    durs = [r['duration'] for r in results if r['duration']]
    print('  总轮数      : %d' % len(results))
    print('  成功轮数    : %d' % n_ok)
    print('  成功率      : %.1f%%' % (100.0 * n_ok / len(results) if results else 0.0))
    if durs:
        print('  平均用时    : %.1f 秒' % (sum(durs) / len(durs)))
        print('  最短 / 最长 : %.1f / %.1f 秒' % (min(durs), max(durs)))
    print()
    for r in results:
        flag = 'OK  ' if r['ok'] else 'FAIL'
        print('  [%s] 第 %2d 轮  %-8s  %s 秒'
              % (flag, r['round'], r['state'] or '-',
                 r['duration'] if r['duration'] is not None else '-'))

    # 保存结果清单
    manifest = os.path.join(log_dir, 'rehearsal_manifest.json')
    with open_text(manifest, 'w') as fp:
        json.dump({'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                   'runs': results}, fp, ensure_ascii=False, indent=2)
    print('\n  清单已写入：%s' % manifest)

    # 自动汇总成 Markdown
    logs = [r['log'] for r in results if r['log'] and os.path.isfile(r['log'])]
    if logs and not args.no_auto_report:
        md_path = os.path.join(log_dir, '实测数据汇总.md')
        generate_markdown(logs, md_path)
        print('  实测数据汇总：%s' % md_path)

    return 0 if n_ok == len(results) else 1


def mode_single(args):
    log_dir = os.path.expanduser(args.runs_dir)
    if not os.path.isdir(log_dir):
        os.makedirs(log_dir)
    ok_i, dur, path, state = run_one_round(1, log_dir, dry_run=args.dry_run,
                                           timeout=args.timeout)
    if os.path.isfile(path):
        print('\n  事件日志：%s' % path)
        print('  查看详情：rosrun carebot_eldercare build_report.py %s' % path)
    return 0 if ok_i else 1


def generate_markdown(logs, out_path):
    """调用 build_report.py 的逻辑生成 Markdown 汇总。"""
    here = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(here, 'build_report.py')
    if not os.path.isfile(script):
        # 安装后位于 lib/<pkg>，尝试从包路径找
        script = 'build_report.py'
    cmd = [sys.executable, script] + logs + ['--markdown', '--out', out_path]
    try:
        subprocess.call(cmd)
    except Exception as exc:
        print('  生成 Markdown 失败：%s' % repr(exc))


def mode_collect(args):
    """只汇总已有日志。"""
    log_dir = os.path.expanduser(args.runs_dir)
    logs = []
    if os.path.isdir(log_dir):
        for name in sorted(os.listdir(log_dir)):
            if name.endswith('.jsonl'):
                logs.append(os.path.join(log_dir, name))
    if not logs:
        print('  在 %s 下没有找到 .jsonl 事件日志' % log_dir)
        return 2

    banner('汇总 %d 个日志文件' % len(logs))
    for p in logs:
        print('  %s' % p)

    md_path = args.markdown or os.path.join(log_dir, '实测数据汇总.md')
    generate_markdown(logs, md_path)
    print('\n  已写入：%s' % md_path)
    return 0


def main():
    setup_stdout()
    parser = argparse.ArgumentParser(
        description='智慧养老护理赛题 — 赛前彩排与数据采集',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=['single', 'repeat', 'collect'],
                        default='single', help='运行模式')
    parser.add_argument('--runs', type=int, default=5, help='repeat 模式的轮数')
    parser.add_argument('--gap', type=float, default=10.0, help='轮间间隔（秒）')
    parser.add_argument('--timeout', type=float, default=300.0, help='单轮超时（秒）')
    parser.add_argument('--runs-dir', default='~/eldercare_runs', help='日志目录')
    parser.add_argument('--dry-run', action='store_true',
                        help='传 dry_run=true 给状态机（不控实物）')
    parser.add_argument('--force', action='store_true', help='跳过 ROS 就绪检查')
    parser.add_argument('--no-auto-report', action='store_true',
                        help='repeat 结束后不自动生成 Markdown')
    parser.add_argument('--markdown', default='', help='collect 模式的输出路径')
    args = parser.parse_args()

    banner('智慧养老护理 — 赛前彩排')
    print('  模式     : %s' % args.mode)
    print('  日志目录 : %s' % os.path.expanduser(args.runs_dir))
    print('  时间     : %s' % time.strftime('%Y-%m-%d %H:%M:%S'))

    if args.mode == 'single':
        return mode_single(args)
    if args.mode == 'repeat':
        return mode_repeat(args)
    return mode_collect(args)


if __name__ == '__main__':
    sys.exit(main())
