#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
preflight_check.py — 智慧养老护理赛题：赛前系统自检

用途
----
比赛现场最可怕的是「上场后才发现某个设备没起来」。这个脚本在赛前跑一遍，
把**设备节点、ROS 话题、关键服务、HomeAssistant 连通性、点位配置**全部检查一遍，
并给出明确的修复建议。

用法
----
    # 只检查设备与配置（不需要 ROS 节点在跑）
    rosrun carebot_eldercare preflight_check.py --devices

    # 检查运行中的系统（需要各节点已启动）
    rosrun carebot_eldercare preflight_check.py

    # 完整检查并输出 JSON 报告
    rosrun carebot_eldercare preflight_check.py --json /tmp/preflight.json

    # 不依赖 roscore 也能跑的纯设备检查
    python preflight_check.py --devices --no-ros

退出码
------
    0 = 全部通过（可能有警告）
    1 = 存在阻塞性问题（红色）
"""

import argparse
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


# ---------------------------------------------------------------------------
# 结果收集
# ---------------------------------------------------------------------------
OK = 'OK'
WARN = 'WARN'
FAIL = 'FAIL'

COLORS = {
    OK:   '\033[32;1m',
    WARN: '\033[33;1m',
    FAIL: '\033[31;1m',
}
RESET = '\033[0m'


def _to_bytes(text):
    """统一转成 UTF-8 字节串，供 Py2 的 str 拼接。

    Py2 下源码里的中文是字节串，而部分 detail 是 unicode；
    一旦某一行先变成 unicode，再拼中文字节串就会按 ASCII 解码并崩：
        UnicodeDecodeError: 'ascii' codec can't decode byte 0xe9
    这个坑会让 `--no-ros` 在打印第一条中文详情时直接退出。
    """
    if isinstance(text, unicode):
        return text.encode('utf-8', 'replace')
    if isinstance(text, bytes):
        return text
    return unicode(text).encode('utf-8', 'replace')


def _to_unicode(text):
    """统一转成 unicode；Py2 下把源码里的 UTF-8 字节串按 UTF-8 解码。"""
    if isinstance(text, bytes):
        return text.decode('utf-8', 'replace')
    if isinstance(text, unicode):
        return text
    return unicode(text)


class Report(object):
    def __init__(self):
        self.items = []

    def add(self, level, category, name, detail='', fix=''):
        self.items.append({
            'level': level, 'category': category,
            'name': name, 'detail': detail, 'fix': fix,
        })

    def ok(self, cat, name, detail=''):
        self.add(OK, cat, name, detail)

    def warn(self, cat, name, detail='', fix=''):
        self.add(WARN, cat, name, detail, fix)

    def fail(self, cat, name, detail='', fix=''):
        self.add(FAIL, cat, name, detail, fix)

    def count(self, level):
        return sum(1 for i in self.items if i['level'] == level)

    def print_all(self):
        current = None
        for item in self.items:
            if item['category'] != current:
                current = item['category']
                print('\n%s── %s %s' % ('\033[36;1m', _to_bytes(current), RESET))
            color = COLORS.get(item['level'], '')
            line = '  %s[%s]%s %s' % (color, item['level'], RESET,
                                      _to_bytes(item['name']))
            if item['detail']:
                line += '\n         %s' % _to_bytes(item['detail'])
            if item['fix'] and item['level'] != OK:
                line += '\n         \033[35m→ 修复：%s\033[0m' % _to_bytes(item['fix'])
            print(line)

    def summary(self):
        return {'ok': self.count(OK), 'warn': self.count(WARN), 'fail': self.count(FAIL)}

    def to_json(self):
        return {'summary': self.summary(), 'items': self.items}


# ---------------------------------------------------------------------------
# 检查项
# ---------------------------------------------------------------------------
def check_devices(rep):
    """检查硬件设备节点是否存在。"""
    devices = [
        ('/dev/stm32',      '底盘 STM32 主控',
         '检查 USB 线是否插好；udev 规则是否已安装并 reload'),
        ('/dev/carebot_laser_ls', '镭神 M10 激光雷达',
         '备选名 /dev/laser（ls01d 雷达）。检查 USB 口与 udev 规则'),
        ('/dev/fdilink_ahrs', 'IMU（fdilink AHRS）',
         '备选名 /dev/art_imu。检查 USB 口与 udev 规则'),
        ('/dev/IOT_net',    '物联网串口网关（备选通道）',
         '未接串口网关时此项可忽略（主通道用 HomeAssistant）'),
    ]
    for path, label, fix in devices:
        if os.path.exists(path):
            try:
                mode = oct(os.stat(path).st_mode & 0o777)
                rep.ok('硬件设备', '%s (%s)' % (label, path), '权限 %s' % mode)
                if not os.access(path, os.R_OK | os.W_OK):
                    rep.warn('硬件设备', '%s 权限不足' % label,
                             '当前用户无法读写 %s' % path,
                             'sudo chmod 666 %s 或把用户加入 dialout 组' % path)
            except Exception as exc:
                rep.warn('硬件设备', label, '无法读取属性：%s' % repr(exc))
        else:
            rep.fail('硬件设备', '%s (%s)' % (label, path), '设备节点不存在', fix)

    # USB 设备（相机 / 麦克风阵列）
    usb_targets = [
        ('2bc5', 'Orbbec Astra 深度相机', '检查 USB 线与供电（相机需要较大电流）'),
        ('10d6', '讯飞六麦阵列（10d6:b003）', '检查 USB 线；阵列需要 5V/0.5A'),
    ]
    try:
        out = subprocess.Popen(['lsusb'], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE).communicate()[0]
        out = out.decode('utf-8', 'replace') if isinstance(out, bytes) else out
        for vid, label, fix in usb_targets:
            if vid in out.lower():
                rep.ok('硬件设备', label, 'lsusb 已识别')
            else:
                rep.fail('硬件设备', label, 'lsusb 未找到 VID %s' % vid, fix)
    except Exception as exc:
        rep.warn('硬件设备', 'lsusb 探测', '无法执行 lsusb：%s' % repr(exc), 'sudo apt-get install usbutils')

    # 音频播放设备
    try:
        out = subprocess.Popen(['aplay', '-l'], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE).communicate()[0]
        out = out.decode('utf-8', 'replace') if isinstance(out, bytes) else out
        if 'card' in out.lower():
            rep.ok('硬件设备', '音频播放设备', out.strip().split('\n')[0][:80])
        else:
            rep.fail('硬件设备', '音频播放设备', 'aplay -l 未列出任何声卡',
                     '检查 3.5mm 扬声器连接；确认声卡驱动已加载')
    except Exception as exc:
        rep.warn('硬件设备', '音频播放设备', '无法执行 aplay：%s' % repr(exc), 'sudo apt-get install alsa-utils')


def check_config_files(rep):
    """检查配置文件是否就绪。"""
    home = os.path.expanduser('~')
    candidates = [
        ('点位配置', os.path.join(home, 'omniWheelCareRobot/rosCode/src/'
                                  'carebot_eldercare/config/waypoints.yaml'),
         '确认点位已按现场地图标定'),
        ('人脸训练器', os.path.join(home, 'omniWheelCareRobot/rosCode/src/'
                                   'face_recognition/script/source/trainer/trainer.yml'),
         '用 face_dataset.py + face_training.py 采集并训练主人的人脸'),
        ('Haar 级联', os.path.join(home, 'omniWheelCareRobot/rosCode/src/'
                                  'face_recognition/script/haarcascade_frontalface_default.xml'), ''),
    ]
    for label, path, fix in candidates:
        if label == '点位配置':
            # 点位配置由 launch 注入，这里只做提示，不作为失败项
            rep.warn('配置', label, '请确认已按现场地图标定（%s）' % label, fix)
            continue
        if os.path.isfile(path):
            size = os.path.getsize(path)
            rep.ok('配置', label, '%s（%d 字节）' % (path, size))
        else:
            rep.fail('配置', label, '文件不存在：%s' % path, fix)


def _load_expected_from_map(path):
    """从 device_map*.yaml 读取 entity_id -> 描述；读不到返回 None。

    这样现场换了设备、改了映射后，不必再改本文件的硬编码表。
    """
    if not path:
        return None
    try:
        import yaml
        with open(path, 'rb') as fp:
            data = yaml.safe_load(fp.read()) or {}
    except Exception:
        return None

    expected = {}
    for group in ('controllable', 'sensors'):
        for name, spec in (data.get(group) or {}).items():
            if not isinstance(spec, dict):
                continue
            eid = spec.get('entity_id')
            if eid:
                expected[eid] = spec.get('desc') or name
    return expected or None


def check_ha(rep, base_url, token, device_map=''):
    """检查 HomeAssistant 连通性与令牌有效性。"""
    if not token:
        rep.fail('物联网', 'HomeAssistant 令牌', '未提供令牌',
                 'export HA_TOKEN=<长期访问令牌>  或写入 /etc/carebot/ha_token')
        return

    try:
        import requests
    except ImportError:
        rep.fail('物联网', 'requests 库', '未安装 requests', 'sudo apt-get install python-requests')
        return

    headers = {'Authorization': 'Bearer %s' % token}
    try:
        t0 = time.time()
        r = requests.get('%s/api/' % base_url.rstrip('/'), headers=headers, timeout=5.0)
        dt = (time.time() - t0) * 1000.0

        # HA 2025.x 的 /api/ 返回 JSON {"message":"API running."}；
        # 老版本返回纯文本 "API running."。两种都算连通，否则新版盒子会误报。
        body = r.text.strip()
        api_ok = body == 'API running.'
        if not api_ok and body.startswith('{'):
            try:
                api_ok = json.loads(body).get('message') == 'API running.'
            except (ValueError, AttributeError):
                api_ok = False

        if r.status_code == 200 and api_ok:
            rep.ok('物联网', 'HomeAssistant 连通性',
                   '%s 响应正常（%.0f ms）' % (base_url, dt))
        elif r.status_code == 401:
            rep.fail('物联网', 'HomeAssistant 令牌', 'HTTP 401 未授权',
                     '令牌已失效，请在 HA 里重新创建长期访问令牌')
            return
        else:
            rep.fail('物联网', 'HomeAssistant 连通性',
                     'HTTP %d：%s' % (r.status_code, r.text[:100]),
                     '检查盒子 IP 与网络')
            return
    except Exception as exc:
        rep.fail('物联网', 'HomeAssistant 连通性', '%s 不可达：%s' % (base_url, exc),
                 '检查盒子是否开机、IP 是否正确、网线/WiFi 是否连通')
        return

    # 枚举全部实体，核对我们的 device_map 是否还存在
    try:
        r = requests.get('%s/api/states' % base_url.rstrip('/'), headers=headers, timeout=10.0)
        states = r.json()
        rep.ok('物联网', '实体枚举', '共 %d 个实体' % len(states))

        expected = _load_expected_from_map(device_map)
        if expected is None:
            expected = {
                'light.0x54ef441000832b4b': '卧室电灯',
                'cover.0x54ef4410004fd6d9': '卧室窗帘',
                'switch.0xa4c138ea9c408dcf': '厨房风扇继电器',
                'switch.0x70b3d52b601499c3': '传感器电源继电器',
                'sensor.0x54ef441000d2f10e_illuminance_lux': '光照强度',
                'sensor.0x00124b002a5183f1_temperature': '温度',
                'sensor.0x00124b002a5183f1_humidity': '湿度',
                'sensor.0x54ef441000d0bded_smoke_density': '烟雾浓度',
            }
        found = dict((s.get('entity_id'), s.get('state')) for s in states)
        for eid, label in expected.items():
            if eid in found:
                rep.ok('物联网', _to_bytes(u'实体 %s' % _to_unicode(label)),
                       _to_bytes(u'%s = %s' % (eid, found[eid])))
            else:
                rep.fail('物联网', _to_bytes(u'实体 %s' % _to_unicode(label)),
                         _to_bytes(u'未找到 %s' % eid),
                         'Zigbee 设备可能已重新配网，entity_id 变了。'
                         '请用 GET /api/states 全量枚举后更新 device_map.yaml')

        # 漏水传感器是资料中唯一缺失的
        leak_like = [s.get('entity_id') for s in states
                     if 'leak' in str(s.get('entity_id', '')).lower()
                     or 'water' in str(s.get('entity_id', '')).lower()]
        if leak_like:
            # r.json() 在 Py2 下返回 unicode entity_id，不能直接拼进
            # 含中文的字节串格式串（会按 ASCII 解码而崩）。
            rep.ok('物联网', '漏水传感器',
                   _to_bytes(u'候选：%s' % u', '.join(leak_like[:5])))
        else:
            rep.warn('物联网', '漏水传感器',
                     '未在 HA 中找到漏水相关实体（书房漏水检测 5 分依赖它）',
                     '确认现场是否有漏水传感器；若无，改用串口网关 ADDR=4 的漏水字段')

        # 顺便把全部可控域列出来，方便现场核对
        domains = {}
        for s in states:
            d = str(s.get('entity_id', '')).split('.')[0]
            domains[d] = domains.get(d, 0) + 1
        rep.ok('物联网', '实体域分布',
               _to_bytes(u', '.join(u'%s×%d' % (k, v)
                                    for k, v in sorted(domains.items()))))
    except Exception as exc:
        rep.warn('物联网', '实体枚举', '枚举失败：%s' % repr(exc))


def check_ros(rep):
    """检查 ROS 图：话题、服务、TF。"""
    try:
        import rospy
    except ImportError:
        rep.fail('ROS', 'rospy', '无法导入 rospy', 'source /opt/ros/kinetic/setup.bash')
        return None

    try:
        master_ok = rospy.get_master().getPid() is not None
    except Exception as exc:
        rep.fail('ROS', 'roscore', '无法连接 ROS Master：%s' % repr(exc),
                 'roslaunch 之前请确认 roscore 已启动')
        return None
    if not master_ok:
        rep.fail('ROS', 'roscore', 'ROS Master 无响应', '启动 roscore')
        return None
    rep.ok('ROS', 'ROS Master', '连接正常')

    try:
        topics = dict((t[0], t[1]) for t in rospy.get_published_topics())
    except Exception as exc:
        rep.fail('ROS', '话题枚举', '失败：%s' % repr(exc))
        return None

    critical_topics = [
        ('/scan',                '激光雷达数据', '雷达驱动未启动（laser_filter.launch）'),
        ('/odom',                '里程计',       '底盘驱动未启动（car_nav.launch）'),
        ('/camera/rgb/image_raw', '相机彩色图',  '相机未启动（astra.launch）'),
        ('/odometry/filtered',   'EKF 融合里程计', 'EKF 未启动（car_nav.launch）'),
        ('/map',                 '地图',         'map_server 未启动或地图路径错误'),
    ]
    for topic, label, fix in critical_topics:
        if topic in topics:
            rep.ok('ROS 话题', '%s (%s)' % (label, topic))
        else:
            rep.fail('ROS 话题', '%s (%s)' % (label, topic), '话题不存在', fix)

    soft_topics = [
        ('/qr/room_order',  '二维码顺序输出', 'qr_detector 未启动'),
        ('/face_rst',       '人脸识别结果',   'face_recognition / face_gate 未启动'),
        ('/iot/state',      '物联网状态',     'iot_bridge 未启动'),
        ('/voice_words',    '语音识别结果',   'voice_bridge 未启动'),
        ('/eldercare/mission_state', '任务状态', 'mission_manager 未启动'),
    ]
    for topic, label, fix in soft_topics:
        if topic in topics:
            rep.ok('ROS 话题', '%s (%s)' % (label, topic))
        else:
            rep.warn('ROS 话题', '%s (%s)' % (label, topic), '话题不存在', fix)

    # 关键服务
    try:
        services = rospy.get_master().getSystemState()[2]
        service_names = [s for group in services for s in group]
    except Exception:
        service_names = []

    critical_services = [
        ('/move_base',        '导航动作服务', 'move_base 未启动'),
        ('/qr/detect_once',   '二维码识别服务', 'qr_detector 未启动'),
        ('/face/verify',      '人脸核验服务', 'face_gate 未启动'),
        ('/iot/refresh',      '物联网刷新服务', 'iot_bridge 未启动'),
        ('/voice/announce',   '语音播报服务', 'voice_bridge 未启动'),
    ]
    for svc, label, fix in critical_services:
        if svc in service_names:
            rep.ok('ROS 服务', '%s (%s)' % (label, svc))
        else:
            rep.warn('ROS 服务', '%s (%s)' % (label, svc), '服务不存在', fix)

    return rospy


def check_tf(rep):
    """检查 TF 树是否完整 —— 这是导航能否工作的根本前提。"""
    try:
        import rospy
        import tf
    except ImportError:
        return
    listener = tf.TransformListener()
    pairs = [
        ('/map', '/odom', '定位（amcl）是否收敛'),
        ('/odom', '/base_footprint', '底盘里程计是否在发布'),
        ('/base_footprint', '/laser_link', '雷达到车体的静态变换'),
    ]
    for parent, child, label in pairs:
        try:
            listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(4.0))
            rep.ok('TF', '%s → %s（%s）' % (parent, child, label))
        except Exception:
            rep.fail('TF', '%s → %s' % (parent, child), '变换不可用',
                     '检查对应驱动是否启动；map→odom 需要 amcl 收敛（在 RViz 里发 2D Pose Estimate）')


def check_waypoints(rep, path):
    """检查点位配置的合理性。"""
    if not path or not os.path.isfile(path):
        rep.warn('点位', '点位文件', '未找到 %s' % path, '确认 launch 里 waypoints_file 参数')
        return
    try:
        import yaml
        with open(path, "rb") as fp:
            data = yaml.safe_load(fp.read()) or {}
        wps = data.get('waypoints') or {}
        if not wps:
            rep.fail('点位', '点位文件', '没有定义任何点位', '填写 waypoints.yaml')
            return
        rep.ok('点位', '点位文件', '共 %d 个点位：%s'
               % (len(wps), ', '.join(sorted(wps.keys()))))

        required = ['qr_area', 'point_A', 'room_B', 'room_C', 'room_D']
        missing = [w for w in required if w not in wps]
        if missing:
            rep.fail('点位', '必需点位缺失', '缺少：%s' % ', '.join(missing),
                     '补齐 waypoints.yaml 中的点位（现场标定）')
        else:
            rep.ok('点位', '必需点位完整', 'qr_area / point_A / room_B / room_C / room_D 均已定义')
    except Exception as exc:
        rep.fail('点位', '点位文件解析', '失败：%s' % repr(exc))


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------
def resolve_token():
    token = os.environ.get('HA_TOKEN', '').strip()
    if token:
        return token, 'env HA_TOKEN'
    for path in ('/etc/carebot/ha_token',):
        if os.path.isfile(path):
            try:
                with open(path) as fp:
                    t = fp.read().strip()
                if t:
                    return t, path
            except Exception:
                pass
    return '', '未找到'


def main():
    setup_stdout()
    parser = argparse.ArgumentParser(
        description='智慧养老护理赛题 — 赛前系统自检')
    parser.add_argument('--devices', action='store_true',
                        help='只检查硬件设备与配置文件')
    parser.add_argument('--no-ros', action='store_true',
                        help='不检查 ROS 图（适用于 roscore 未启动时）')
    parser.add_argument('--json', default='', help='把结果写入 JSON 文件')
    parser.add_argument('--ha-url', default='http://192.168.8.190:8123',
                        help='HomeAssistant 地址')
    parser.add_argument('--waypoints', default='', help='点位配置文件路径')
    parser.add_argument('--device-map', default='',
                        help='device_map*.yaml 路径；给出后按它核对实体，'
                             '不再用脚本里的出厂硬编码表')
    # 原来直接写 rospy.myargv()，但本文件顶部并没有 import rospy；
    # --no-ros 时会在解析参数前就 NameError。这里按需导入并退化到 sys.argv。
    try:
        import rospy
        argv = rospy.myargv()[1:]
    except ImportError:
        argv = sys.argv[1:]
    args = parser.parse_args(argv)

    rep = Report()

    print('\033[36;1m')
    print('=' * 68)
    print('  智慧养老护理赛题 — 赛前系统自检')
    print('  时间：%s' % time.strftime('%Y-%m-%d %H:%M:%S'))
    print('=' * 68)
    print('\033[0m')

    # 1) 设备
    check_devices(rep)

    # 2) 配置
    check_config_files(rep)

    # 3) HomeAssistant
    token, source = resolve_token()
    if token:
        rep.ok('物联网', '令牌来源', source)
    check_ha(rep, args.ha_url, token, args.device_map)

    # 4) ROS
    if not args.devices and not args.no_ros:
        check_ros(rep)
        check_tf(rep)

    # 5) 点位
    check_waypoints(rep, args.waypoints)

    # 输出
    rep.print_all()
    s = rep.summary()
    print('\n' + '=' * 68)
    color = COLORS[FAIL] if s['fail'] else (COLORS[WARN] if s['warn'] else COLORS[OK])
    print('%s  结果：通过 %d 项，警告 %d 项，失败 %d 项  %s'
          % (color, s['ok'], s['warn'], s['fail'], RESET))
    print('=' * 68)

    if s['fail']:
        print('\n\033[31;1m存在阻塞性问题，请先按上面的「→ 修复」提示处理后再比赛。\033[0m')
    elif s['warn']:
        print('\n\033[33;1m没有阻塞性问题，但有警告项，建议逐条确认。\033[0m')
    else:
        print('\n\033[32;1m全部检查通过，可以开始比赛。\033[0m')

    if args.json:
        try:
            # ⚠️ 必须**二进制 + 显式 UTF-8**。
            #
            # Py2 下 `open(path, 'w')` 是 ASCII 文本模式，
            # 而 `ensure_ascii=False` 会产生含中文的 unicode
            # → UnicodeEncodeError。报告里有大量中文，必然踩。
            #
            # 同类 bug 在 `mission_manager.EventLogger.log` 上真实发生过，
            # 而且被 `except: pass` 吞掉 —— 事件日志从第一条中文起全部丢失，
            # 表现得像"任务卡死在导航开始处"，我为此查了好几轮。
            # 现已用 `_tools/check_py2_file_writes.py` 全项目扫过。
            with open(args.json, 'wb') as fp:
                fp.write(json.dumps(rep.to_json(), ensure_ascii=False,
                                    indent=2).encode('utf-8'))
            print('JSON 报告已写入：%s' % args.json)
        except Exception as exc:
            print('写入 JSON 报告失败：%s' % repr(exc))

    return 1 if s['fail'] else 0


if __name__ == '__main__':
    sys.exit(main())
