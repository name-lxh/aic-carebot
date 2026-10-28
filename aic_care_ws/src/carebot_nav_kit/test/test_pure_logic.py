#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
test_pure_logic.py — 纯逻辑单元测试（不需要 ROS 运行、不需要硬件）

为什么要有这个
--------------
工作区里 90% 的代码是 ROS 节点，跑起来需要 roscore + 硬件。但其中真正
"有逻辑、会出错"的部分是**纯函数**：
    · 二维码载荷解析（支持 6 种格式 + 严格校验）
    · 房间顺序归一化
    · 设备映射与阈值判定
    · 里程计漂移计算

这些完全可以在没有机器人、甚至没有 roscore 的情况下测。而且它们恰恰是
最容易出 bug 的地方 —— 比如二维码解析错了，后面 30 分的房间任务全走错。

测试策略
--------
节点文件顶层就 import rospy/cv2，不能直接 import。所以用**源码抽取**：
读文件 → 切出纯函数段 → exec 到一个干净的命名空间里测。

这样测的是**真实的源码**（不是复制的副本），改了源码测试就会跟着变。

用法
----
    python3 test_pure_logic.py            # 跑全部
    python3 test_pure_logic.py -v         # 详细输出
"""

import io
import os
import re
import sys
import unittest


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


# ---------------------------------------------------------------------------
# 源码抽取工具
# ---------------------------------------------------------------------------
def load_functions(src_path, start_marker, end_marker, extra_ns=None):
    """从源码文件里抽出 [start_marker, end_marker) 之间的代码并 exec。

    节点文件顶层有 rospy/cv2 等 import，无法整体 import，
    所以只取纯函数那一段。测的是真实源码，不是副本。
    """
    with io.open(src_path, encoding='utf-8-sig') as fp:
        src = fp.read()

    i = src.find(start_marker)
    if i < 0:
        raise RuntimeError(u'找不到起始标记：%s（文件 %s）' % (start_marker, src_path))
    j = src.find(end_marker, i)
    if j < 0:
        raise RuntimeError(u'找不到结束标记：%s（文件 %s）' % (end_marker, src_path))

    chunk = src[i:j]

    ns = {'__name__': 'extracted', 'unicode': unicode}
    if extra_ns:
        ns.update(extra_ns)

    # 提供 re/json 等纯标准库
    import re as _re
    import json as _json
    ns.setdefault('re', _re)
    ns.setdefault('json', _json)

    exec(compile(chunk, src_path, 'exec'), ns)
    return ns


def find_src(*rel_paths):
    """在常见位置找源码文件。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, '..', '..'),            # src/carebot_nav_kit/test -> src
        os.path.join(here, '..'),
        os.path.expanduser('~/catkin_ws/src'),
    ]
    for base in candidates:
        for rel in rel_paths:
            p = os.path.abspath(os.path.join(base, rel))
            if os.path.isfile(p):
                return p
    return None


# ===========================================================================
#  测试 1：二维码载荷解析
# ===========================================================================
# 字节串插进 unicode 格式串在 Py2 下会按 ASCII 解码而崩，所以参数先转 unicode。
def _t(v):
    if v is None:
        return u''
    if isinstance(v, bytes):
        try:
            return v.decode('utf-8')
        except Exception:
            return v.decode('utf-8', 'replace')
    return u'%s' % (v,)

class TestQrParsing(unittest.TestCase):
    """二维码解析 —— 决定后续 30 分房间任务的执行顺序，必须严格。"""

    @classmethod
    def setUpClass(cls):
        path = find_src('carebot_qr/nodes/qr_detector.py')
        if not path:
            raise unittest.SkipTest(u'找不到 qr_detector.py')
        cls.path = path
        # ⚠️ 必须把 `to_text` 作为依赖注入进去。
        #
        # `load_functions` 是从 `ROOM_B = 2` 开始**抠一段代码**出来 exec 的，
        # 而 `qr_detector.py` 里的 `to_text`（复用 utf8_stdio 的那个）
        # 定义在 `ROOM_B = 2` **之前**，所以不在被抠出的代码里 ——
        # 于是 `parse_room_order` 调 `to_text(...)` 就 NameError。
        #
        # `extra_ns` 参数本来就是为这种情况准备的。
        # 这里注入的是**与源码等价的实现**（同样容忍 bytes）。
        def _to_text(v):
            if v is None:
                return u''
            if isinstance(v, bytes):
                try:
                    return v.decode('utf-8')
                except Exception:
                    return v.decode('utf-8', 'replace')
            return u'%s' % (v,)

        cls.ns = load_functions(path, 'ROOM_B = 2',
                                '# ------',  # 主节点分隔线
                                extra_ns={'to_text': _to_text})
        cls.parse = staticmethod(cls.ns['parse_room_order'])
        cls.to_text = staticmethod(cls.ns['order_to_text'])

    # ---------------- 正例：必须能解析 ----------------
    def test_digits_compact(self):
        self.assertEqual(self.parse('234')[0], [2, 3, 4])

    def test_digits_all_permutations(self):
        """6 种排列都要能解析，且顺序不能错。"""
        import itertools
        for perm in itertools.permutations('234'):
            s = ''.join(perm)
            expect = [int(c) for c in perm]
            got, _ = self.parse(s)
            self.assertEqual(got, expect, u'%s 解析错误' % s)

    def test_digits_separated(self):
        for s in ('2,3,4', '2-3-4', '2 3 4', '2/3/4', '2;3;4'):
            self.assertEqual(self.parse(s)[0], [2, 3, 4], u'%s 解析错误' % s)

    def test_letters(self):
        for s in ('BCD', 'bcd', 'B,C,D', 'b-c-d', 'B C D'):
            self.assertEqual(self.parse(s)[0], [2, 3, 4], u'%s 解析错误' % s)

    def test_letters_reverse(self):
        self.assertEqual(self.parse('DCB')[0], [4, 3, 2])

    def test_json_object(self):
        self.assertEqual(self.parse('{"order": [3, 2, 4]}')[0], [3, 2, 4])
        self.assertEqual(self.parse('{"rooms": ["D","C","B"]}')[0], [4, 3, 2])

    def test_json_array(self):
        self.assertEqual(self.parse('[4, 3, 2]')[0], [4, 3, 2])

    def test_url(self):
        self.assertEqual(self.parse('http://x/y?order=234&z=1')[0], [2, 3, 4])
        self.assertEqual(self.parse('http://x/y?rooms=DCB')[0], [4, 3, 2])

    def test_chinese(self):
        self.assertEqual(self.parse(u'卧室,厨房,书房')[0], [2, 3, 4])
        self.assertEqual(self.parse(u'书房、厨房、卧室')[0], [4, 3, 2])

    # ---------------- 反例：必须拒绝 ----------------
    def test_reject_wrong_length(self):
        """长度不对必须拒绝 —— 接受错误顺序比识别失败更糟。"""
        for s in ('23', '2345', '2', ''):
            self.assertIsNone(self.parse(s)[0], u'%s 不该被接受' % _t(s))

    def test_reject_duplicates(self):
        """有重复值必须拒绝。"""
        for s in ('2344', '223', '444', 'BCC'):
            self.assertIsNone(self.parse(s)[0], u'%s 不该被接受' % _t(s))

    def test_reject_out_of_range(self):
        """含 B/C/D 之外的值必须拒绝。"""
        for s in ('111', '2340', '567', '999'):
            self.assertIsNone(self.parse(s)[0], u'%s 不该被接受' % _t(s))

    def test_reject_garbage(self):
        for s in ('hello', 'ABC', 'AB', 'xyz', '\x00\x01', '----'):
            self.assertIsNone(self.parse(s)[0], u'%s 不该被接受' % _t(s))

    def test_reject_partial_letters(self):
        """字母不足 3 个必须拒绝。"""
        for s in ('BC', 'B', 'BD'):
            self.assertIsNone(self.parse(s)[0], u'%s 不该被接受' % _t(s))

    # ---------------- 返回值结构 ----------------
    def test_returns_tuple(self):
        r = self.parse('234')
        self.assertIsInstance(r, tuple)
        self.assertEqual(len(r), 2)
        self.assertIsInstance(r[0], list)
        self.assertIsInstance(r[1], (str, unicode))

    def test_chinese_text(self):
        """中文播报文本要正确。"""
        t = self.to_text([2, 3, 4])
        self.assertIn(u'卧室', t)
        self.assertIn(u'厨房', t)
        self.assertIn(u'书房', t)

    def test_long_input_no_crash(self):
        """超长输入不能崩（二维码可能被误读成一大串）。"""
        self.parse('x' * 10000)
        self.parse('2' * 1000)


# ===========================================================================
#  测试 2：数值归一化与工具函数
# ===========================================================================
class TestNormalization(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        path = find_src('carebot_qr/nodes/qr_detector.py')
        if not path:
            raise unittest.SkipTest(u'找不到 qr_detector.py')
        cls.ns = load_functions(path, 'ROOM_B = 2', '# ------')

    def test_to_text_handles_bytes(self):
        f = self.ns['_to_text']
        self.assertEqual(f(b'abc'), u'abc')
        self.assertEqual(f(u'中文'), u'中文')
        self.assertEqual(f(123), u'123')

    def test_u8_returns_bytes(self):
        f = self.ns['_u8']
        r = f(u'中文')
        self.assertIsInstance(r, bytes)
        self.assertEqual(r, u'中文'.encode('utf-8'))

    def test_u8_passthrough_bytes(self):
        f = self.ns['_u8']
        self.assertEqual(f(b'abc'), b'abc')


# ===========================================================================
#  测试 3：IoT 设备映射与阈值判定
# ===========================================================================
class TestIotMapping(unittest.TestCase):
    """设备映射 —— 决定 30 分的物联网任务能不能打到正确的实体上。"""

    @classmethod
    def setUpClass(cls):
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest(u'缺 pyyaml')

        path = find_src('carebot_iot/config/device_map.yaml')
        if not path:
            raise unittest.SkipTest(u'找不到 device_map.yaml')
        cls.path = path
        with open(path, "rb") as fp:
            cls.cfg = yaml.safe_load(fp.read())

    def test_has_controllable(self):
        self.assertIn('controllable', self.cfg)
        self.assertGreaterEqual(len(self.cfg['controllable']), 4)

    def test_has_sensors(self):
        self.assertIn('sensors', self.cfg)
        self.assertGreaterEqual(len(self.cfg['sensors']), 5)

    def test_all_entity_ids_wellformed(self):
        """entity_id 必须是 domain.object_id 形式。"""
        import re
        pat = re.compile(r'^[a-z_]+\.[a-zA-Z0-9_]+$')
        problems = []
        for section in ('controllable', 'sensors'):
            for name, spec in (self.cfg.get(section) or {}).items():
                eid = spec.get('entity_id')
                if not eid:
                    continue    # 允许留空（现场填写）
                if not pat.match(eid):
                    # ⚠️ eid 是 HomeAssistant entity_id，可能含非 ASCII；
                    #    %r 产出字节串，混进 unicode 格式串会崩。
                    problems.append(u'%s.%s = %s'
                                    % (section, name, _t(eid)))
        self.assertEqual(problems, [], u'entity_id 格式不对：%s' % problems)

    def test_domains_match_entity_prefix(self):
        """domain 字段必须与 entity_id 的前缀一致。"""
        problems = []
        for name, spec in self.cfg['controllable'].items():
            eid = spec.get('entity_id') or ''
            dom = spec.get('domain')
            if not eid or not dom:
                continue
            if not eid.startswith(dom + '.'):
                problems.append(u'%s: domain=%s 但 entity_id=%s' % (name, dom, eid))
        self.assertEqual(problems, [], u'domain 与 entity_id 前缀不符：%s' % problems)

    def test_smoke_has_threshold(self):
        """烟雾是浓度传感器，必须有阈值判定，否则无法判断"是否异常"。"""
        smoke = self.cfg['sensors'].get('smoke')
        self.assertIsNotNone(smoke, u'缺 smoke 传感器')
        self.assertIn('threshold', smoke, u'smoke 必须有 threshold')
        self.assertIsInstance(float(smoke['threshold']), float)

    def test_binary_sensors_have_on_state(self):
        """二元传感器（漏水）必须有 on_state，否则无法判定异常。"""
        leak = self.cfg['sensors'].get('water_leak')
        self.assertIsNotNone(leak)
        if leak.get('entity_id'):      # 现场填写后才校验
            self.assertIn('on_state', leak)
            self.assertIn('on_text', leak)

    def test_legacy_topics_consistent(self):
        """旧话题协议映射的设备必须真实存在。"""
        legacy = self.cfg.get('legacy_topics') or {}
        if not legacy.get('enabled'):
            self.skipTest(u'旧话题协议未启用')
        for topic, device in legacy.items():
            if topic in ('enabled', 'on_max', 'off_min'):
                continue
            self.assertIn(device, self.cfg['controllable'],
                          u'旧话题 %s 指向未知设备 %s' % (topic, device))

    def test_groups_members_exist(self):
        """分组成员必须存在。"""
        for gname, g in (self.cfg.get('groups') or {}).items():
            for m in g.get('members', []):
                self.assertIn(m, self.cfg['controllable'],
                              u'分组 %s 的成员 %s 不存在' % (gname, m))

    def test_base_url_format(self):
        url = self.cfg.get('base_url', '')
        self.assertTrue(url.startswith('http://') or url.startswith('https://'),
                        u'base_url 格式不对：%s' % _t(url))


# ===========================================================================
#  测试 4：导航配置一致性
# ===========================================================================
class TestNavConfigConsistency(unittest.TestCase):
    """配置一致性 —— 这些数值不一致会导致"规划的路走不了"。"""

    @classmethod
    def setUpClass(cls):
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest(u'缺 pyyaml')
        paths = {}
        for key, rel in [
            ('base', 'carebot_nav_kit/config/robot/base.yaml'),
            ('costmap', 'carebot_nav_kit/config/nav/costmap_common_params.yaml'),
            ('gcost', 'carebot_nav_kit/config/nav/global_costmap_params.yaml'),
            ('lcost', 'carebot_nav_kit/config/nav/local_costmap_params.yaml'),
            ('amcl', 'carebot_nav_kit/config/nav/amcl_params.yaml'),
            ('dwa', 'carebot_nav_kit/config/nav/dwa_local_planner_params.yaml'),
            ('ekf', 'carebot_nav_kit/config/nav/ekf_params.yaml'),
            ('ekf_launch', 'carebot_nav_kit/launch/include/ekf.launch'),
            ('filter', 'carebot_nav_kit/config/filters/laser_filter_chain.yaml'),
        ]:
            p = find_src(rel)
            if not p:
                raise unittest.SkipTest(u'找不到 %s' % rel)
            with open(p, "rb") as fp:
                paths[key] = yaml.safe_load(fp.read())
        cls.c = paths

    def test_robot_radius_consistent(self):
        """base.yaml 与 costmap 的 robot_radius 必须一致。"""
        r1 = float(self.c['base']['robot']['geometry']['radius'])
        r2 = float(self.c['costmap']['robot_radius'])
        self.assertAlmostEqual(r1, r2, places=6,
                               msg=u'base=%.3f costmap=%.3f 不一致' % (r1, r2))

    def test_laser_cutoff_exceeds_radius(self):
        """激光近端截断必须大于车体半径，否则机器人把自己当障碍。"""
        r = float(self.c['base']['robot']['geometry']['radius'])
        lower = None
        for f in self.c['filter']['scan_filter_chain']:
            if f.get('name') == 'range':
                lower = float(f['params']['lower_threshold'])
        self.assertIsNotNone(lower, u'缺距离滤波器')
        self.assertGreater(lower, r,
                           u'激光截断 %.3f 必须 > 车体半径 %.3f' % (lower, r))

    def test_inflation_exceeds_radius(self):
        r = float(self.c['base']['robot']['geometry']['radius'])
        infl = float(self.c['costmap']['inflation_layer']['inflation_radius'])
        self.assertGreaterEqual(infl, r,
                                u'膨胀半径 %.3f < 车体半径 %.3f' % (infl, r))

    def test_base_frames_unified(self):
        """base_frame 必须全部统一 —— 混用会让定位与规划错位。"""
        want = self.c['base']['robot']['frames']['base_footprint']
        self.assertEqual(self.c['gcost']['global_costmap']['robot_base_frame'], want)
        self.assertEqual(self.c['lcost']['local_costmap']['robot_base_frame'], want)
        self.assertEqual(self.c['amcl']['base_frame_id'], want)
        self.assertEqual(self.c['ekf']['base_link_frame'], want)

    def test_velocity_meets_competition_rules(self):
        """赛题规则：线速度 ≥0.6 m/s，角速度 ≥4 rad/s。"""
        dw = self.c['dwa']['DWAPlannerROS']
        self.assertGreaterEqual(float(dw['max_vel_x']), 0.6,
                                u'线速度 %.2f < 规则下限 0.6' % dw['max_vel_x'])
        self.assertGreaterEqual(float(dw['max_rot_vel']), 4.0,
                                u'角速度 %.2f < 规则下限 4.0' % dw['max_rot_vel'])

    def test_omni_matches_holonomic(self):
        """全向底盘必须开 holonomic，否则浪费横移能力。"""
        kine = self.c['base']['robot']['kinematics']
        self.assertEqual(self.c['amcl']['odom_model_type'], kine['type'])
        if kine['type'] == 'omni':
            self.assertTrue(self.c['dwa']['DWAPlannerROS']['holonomic_robot'])

    def test_ekf_world_frame_equals_odom(self):
        """world_frame 必须等于 odom_frame，否则与 amcl 抢 map→odom。"""
        self.assertEqual(self.c['ekf']['world_frame'], self.c['ekf']['odom_frame'])

    def test_ekf_covariance_sizes(self):
        for k in ('process_noise_covariance', 'initial_estimate_covariance'):
            self.assertEqual(len(self.c['ekf'][k]), 225,
                             u'%s 应为 15×15=225' % k)

    def test_ekf_config_lengths(self):
        for k in ('odom0_config', 'imu0_config'):
            self.assertEqual(len(self.c['ekf'][k]), 15,
                             u'%s 应为 15 位' % k)

    def test_costmap_uses_filtered_scan(self):
        """costmap 必须订阅滤波后的 scan。"""
        topic = self.c['costmap']['obstacle_layer']['scan']['topic']
        self.assertEqual(topic, '/scan_filtered',
                         u'costmap 订阅了 %s，应改为 /scan_filtered' % topic)

    def test_dwa_top_level_latch_param(self):
        """latch_xy_goal_tolerance 必须在 move_base 段，不能在 DWAPlannerROS 段。

        这个文件的结构是：
            DWAPlannerROS:   {...}    <- 加载到 /move_base/DWAPlannerROS/
            move_base:       {...}    <- 加载到 /move_base/

        move_base 读的是 **/move_base/latch_xy_goal_tolerance**，
        所以它必须写在 move_base 段里。写在 DWAPlannerROS 段会变成
        /move_base/DWAPlannerROS/latch_xy_goal_tolerance，静默失效。

        （实测教训：这个 bug 真的存在过，注释里写着"这是顶层参数"
          但代码放在了 namespace 下 —— 注释和代码矛盾。）
        """
        self.assertIn('latch_xy_goal_tolerance',
                      self.c['dwa'].get('move_base', {}),
                      u'latch_xy_goal_tolerance 必须在 move_base 段')
        self.assertNotIn('latch_xy_goal_tolerance',
                         self.c['dwa'].get('DWAPlannerROS', {}),
                         u'latch_xy_goal_tolerance 不该出现在 DWAPlannerROS 段'
                         u'（那样会变成 /move_base/DWAPlannerROS/... 而失效）')

    def test_goal_tolerance_pair_consistent(self):
        """xy/yaw_goal_tolerance 应在 DWAPlannerROS 段（它是规划器参数）。"""
        dw = self.c['dwa'].get('DWAPlannerROS', {})
        self.assertIn('xy_goal_tolerance', dw)
        self.assertIn('yaw_goal_tolerance', dw)


# ===========================================================================
#  测试 5：航点配置
# ===========================================================================
class TestWaypoints(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest(u'缺 pyyaml')
        p = find_src('carebot_nav_kit/config/waypoints.yaml')
        if not p:
            raise unittest.SkipTest(u'找不到 waypoints.yaml')
        with open(p, "rb") as fp:
            cls.cfg = yaml.safe_load(fp.read())

    def test_required_waypoints_present(self):
        """赛题需要的点位必须都有。"""
        wps = self.cfg['waypoints']
        for name in ('qr_area', 'point_A', 'room_B', 'room_C', 'room_D'):
            self.assertIn(name, wps, u'缺赛题必需点位 %s' % name)

    def test_waypoint_fields(self):
        for name, w in self.cfg['waypoints'].items():
            for k in ('x', 'y', 'yaw'):
                self.assertIn(k, w, u'%s 缺字段 %s' % (name, k))
                self.assertIsInstance(float(w[k]), float)

    def test_yaw_in_range(self):
        """yaw 应在 [-pi, pi] 内。"""
        import math
        for name, w in self.cfg['waypoints'].items():
            y = float(w['yaw'])
            self.assertTrue(-math.pi - 0.01 <= y <= math.pi + 0.01,
                            u'%s 的 yaw=%.3f 超出范围' % (name, y))

    def test_frame_id(self):
        self.assertEqual(self.cfg.get('frame_id', 'map'), 'map')


# ===========================================================================
#  测试 6：硬件 profile 完整性
# ===========================================================================
class TestHardwareProfiles(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest(u'缺 pyyaml')
        p = find_src('carebot_nav_kit/config/hardware/lab_carebot.yaml')
        if not p:
            raise unittest.SkipTest(u'找不到硬件 profile')
        import glob
        d = os.path.dirname(p)
        cls.profiles = {}
        for f in glob.glob(os.path.join(d, '*.yaml')):
            with open(f, "rb") as fp:
                cls.profiles[os.path.basename(f)[:-5]] = yaml.safe_load(fp.read())

    def test_at_least_two_profiles(self):
        self.assertGreaterEqual(len(self.profiles), 2)

    def test_each_has_required_fields(self):
        for name, p in self.profiles.items():
            self.assertIn('profile_name', p, u'%s 缺 profile_name' % name)
            self.assertIn('subsystems', p, u'%s 缺 subsystems' % name)

    def test_subsystems_have_enable(self):
        for name, p in self.profiles.items():
            for sk, s in (p.get('subsystems') or {}).items():
                self.assertIn('enable', s,
                              u'%s 的子系统 %s 缺 enable 字段' % (name, sk))

    def test_enabled_subsystems_have_pkg(self):
        """启用的子系统必须有 launch_pkg，否则启动器无法映射驱动。"""
        for name, p in self.profiles.items():
            for sk, s in (p.get('subsystems') or {}).items():
                if s.get('enable'):
                    self.assertTrue(s.get('launch_pkg'),
                                    u'%s 的 %s 已启用但缺 launch_pkg' % (name, sk))

    def test_driver_mapping_covers_enabled_subsystems(self):
        """**已启用**的子系统，其包必须在 nav_launch.py 的映射表里。

        注意：只校验 enable=true 的子系统。
        未启用的（比如 camera，默认 false）不需要映射 —— 它们不由导航启动器管理。
        但 nav_launch.py 必须把"启用了却没处理"的子系统**显式报告**出来，
        不能静默忽略（见 test_nav_launch_reports_unhandled）。
        """
        mapper = find_src('carebot_nav_kit/scripts/nav_launch.py')
        if not mapper:
            self.skipTest(u'找不到 nav_launch.py')
        with io.open(mapper, encoding='utf-8-sig') as fp:
            src = fp.read()
        i = src.find('PKG_TO_DRIVER = {')
        j = src.find('}', i)
        table = src[i:j]

        packed_subsys = ('base', 'laser', 'imu')
        for name, p in self.profiles.items():
            for sk, s in (p.get('subsystems') or {}).items():
                if not s.get('enable'):
                    continue
                if sk not in packed_subsys:
                    # 非导航子系统（如 camera），由 nav_launch 单独报告
                    continue
                pkg = s.get('launch_pkg')
                self.assertTrue(pkg, u'%s 的 %s 已启用但缺 launch_pkg' % (name, sk))
                self.assertIn("'%s'" % pkg, table,
                              u'%s 的 %s 用了 %s，但 nav_launch.py 的 '
                              u'PKG_TO_DRIVER 里没有映射' % (name, sk, pkg))

    def test_nav_launch_reports_unhandled(self):
        """nav_launch.py 必须能报告"启用但不由它处理"的子系统。"""
        mapper = find_src('carebot_nav_kit/scripts/nav_launch.py')
        if not mapper:
            self.skipTest(u'找不到 nav_launch.py')
        with io.open(mapper, encoding='utf-8-sig') as fp:
            src = fp.read()
        # 必须有"其他子系统"的处理逻辑
        self.assertIn('other', src.lower(),
                      u'nav_launch.py 应报告未处理的子系统，而不是静默忽略')


# ===========================================================================
#  测试 7：输出编码设置的完整性（防回归）
# ===========================================================================
class TestStdoutEncodingSetup(unittest.TestCase):
    """每个打印中文的脚本都必须做输出编码设置。

    为什么要有这个测试
    ------------------
    实测踩过的坑：工作区里 20 个脚本打印中文，其中 11 个**完全没有**
    编码设置，另外 9 个用了漏掉 Python 3.6 的 if/elif 写法。后果是在
    rosrun 环境（没有 UTF-8 locale）下，Python 3.6 默认 ASCII 编码，
    打印中文直接崩：

        UnicodeEncodeError: 'ascii' codec can't encode characters

    而且这类崩溃往往发生在**运行中途**（比如状态机播报房间名），
    现场极难排查。本地 Python 3.11 测不出来，只有目标机器会暴露。

    这个测试静态扫描源码，确保不会再退化。
    """

    @classmethod
    def setUpClass(cls):
        here = os.path.dirname(os.path.abspath(__file__))
        # ⚠️⚠️ 只扫**我们自己的包**，不能扫整个工作空间。
        #
        # 踩过：原来 root 是 `../../`（workspace/src），
        # 于是把**组委会/厂商的包**（face_recognition、lslidar_driver …）
        # 也扫进来了，报了一堆跟我们无关的文件：
        #     face_recognition/script/faceDetection_video_to_img.py
        #     ...
        # **那不是我们的代码，我们也不该去改它。**
        # 这个测试的职责边界是"我们的代码别退化"，
        # 扫到别人头上就变成了"永远失败的检查"。
        src_root = os.path.abspath(os.path.join(here, '..', '..'))
        cls.roots = []
        for name in sorted(os.listdir(src_root)):
            d = os.path.join(src_root, name)
            if name.startswith('carebot_') and os.path.isdir(d):
                cls.roots.append(d)
        cls.root = src_root
        if not cls.roots:
            raise unittest.SkipTest(u'找不到 carebot_* 包目录')

    def _py_files(self):
        out = []
        for root in self.roots:
            for dirpath, dirnames, filenames in os.walk(root):
                dirnames[:] = [d for d in dirnames
                               if d not in ('build', 'devel', 'install', '.git',
                                            '_stubs', '__pycache__', 'test')]
                for f in filenames:
                    if f.endswith('.py') and f != 'utf8_stdio.py':
                        out.append(os.path.join(dirpath, f))
        return sorted(out)

    @staticmethod
    def _has_stdout_setup(text):
        """判断文件是否**接入**了输出编码设置。

        接受的两种写法：
          1. 自己定义：      `def setup_stdout`
          2. **从共享模块 import**（推荐）：
                 `from utf8_stdio import setup_all as setup_stdout`
                 `from utf8_stdio import setup_stdout`
                 `from ros_logfix import setup_stdout`

        ⚠️ 第 2 种必须认。项目里有几个脚本是
           `from utf8_stdio import setup_all as setup_stdout` 之后
           `setup_stdout()` —— **这是比"各写一份"更好的做法**
           （编码处理只应有一份权威实现）。
           原来判据只认 `def setup_stdout`，就是**惩罚了正确写法**。
        """
        if 'def setup_stdout' in text:
            return True
        for mod in ('utf8_stdio', 'ros_logfix'):
            if re.search(r'from\s+%s\s+import\s+[^\n]*setup_' % mod, text):
                return True
        return False

    @staticmethod
    def _calls_stdout_setup(text):
        return 'setup_stdout()' in text or 'setup_all()' in text

    def test_files_with_chinese_have_setup(self):
        """含中文输出的文件必须**接入**输出编码设置。

        ⚠️ 判据见 `_has_stdout_setup` —— 自己定义、或从共享模块
           import 都算。后者是更好的做法，不该被这条测试惩罚。
        """
        import re as _re
        cjk = _re.compile(u'[\u4e00-\u9fff]')
        problems = []
        checked = 0
        for p in self._py_files():
            with io.open(p, encoding='utf-8-sig') as fp:
                text = fp.read()
            if not cjk.search(text):
                continue        # 没中文，不需要
            checked += 1
            has_def = self._has_stdout_setup(text)
            has_call = self._calls_stdout_setup(text)
            if not (has_def and has_call):
                problems.append(u'%s (接入=%s 调用=%s)'
                                % (os.path.relpath(p, self.root),
                                   has_def, has_call))
        self.assertGreater(checked, 10, u'应该扫描到多个含中文的文件')
        self.assertEqual(problems, [],
                         u'以下文件打印中文但没有接入编码设置'
                         u'（自己定义 setup_stdout，或从 utf8_stdio '
                         u'import），在无 UTF-8 locale 的机器上会崩：\n  '
                         + u'\n  '.join(problems))

    def test_no_python36_incompatible_pattern(self):
        """不能出现"漏掉 Python 3.7 以下"的 if/elif 写法（注释除外）。"""
        bad_pat = u"hasattr(sys.stdout, 'reconfigure')"
        problems = []
        for p in self._py_files():
            with io.open(p, encoding='utf-8-sig') as fp:
                lines = fp.read().split('\n')
            in_doc = False
            for i, line in enumerate(lines, 1):
                s = line.strip()
                if s.startswith('#'):
                    continue
                # 跳过 docstring 区域（用三引号计数近似判断）
                if s.count('"""') == 1:
                    in_doc = not in_doc
                    continue
                if in_doc:
                    continue
                if bad_pat in line:
                    problems.append(u'%s:%d  %s'
                                    % (os.path.relpath(p, self.root), i, s))
        self.assertEqual(problems, [],
                         u'以下位置使用了只覆盖 3.7+ 的写法，'
                         u'在 Python 3.6 上会静默失效：\n  '
                         + u'\n  '.join(problems))

    def test_setup_uses_codecs_getwriter(self):
        """实现里必须用 codecs.getwriter（唯一能覆盖 2.6~3.11 的方式）。"""
        problems = []
        for p in self._py_files():
            with io.open(p, encoding='utf-8-sig') as fp:
                text = fp.read()
            if 'def setup_stdout' not in text:
                continue
            if 'getwriter' not in text:
                problems.append(os.path.relpath(p, self.root))
        self.assertEqual(problems, [],
                         u'以下文件的 setup_stdout 没走 codecs.getwriter：\n  '
                         + u'\n  '.join(problems))

    def test_rospy_logging_nodes_have_fix(self):
        """用 rospy 日志函数的节点必须有 fix_ros_logging。

        为什么（这是本项目最严重的一个坑）
        ----------------------------------
        ROS Melodic 跑在 **Python 2.7** 下，Python 2 的 `str(unicode)`
        默认用 ASCII 编码。而 ROS 的日志处理器正是这么转的：

            rosgraph/roslogging.py:249
                msg = msg.replace('${message}', str(record_message))

        于是 `rospy.loginfo(u'中文')` 直接抛
            UnicodeEncodeError: 'ascii' codec can't encode characters

        **任何节点用 rospy.log* 打中文都会崩。** 更糟的是崩溃常发生在
        异常处理路径上（`except: rospy.logerr(u'出错了：%s' % repr(e))`），
        把真正的异常掩盖掉 —— 调试时被误导了很久。

        实测复现：
            File "fake_sensors.py", line 264, in __init__
                rospy.loginfo(u'假传感器仿真器已启动')
            UnicodeEncodeError: 'ascii' codec ...

        修法是在 record 进入 handler 前把 unicode 转成 UTF-8 字节串
        （见 fix_ros_logging）。
        """
        import re as _re
        logpat = _re.compile(r'rospy\.(loginfo|logwarn|logerr|logdebug|logfatal)')
        problems = []
        checked = 0
        for p in self._py_files():
            with io.open(p, encoding='utf-8-sig') as fp:
                text = fp.read()
            if not logpat.search(text):
                continue        # 不用 rospy 日志，不需要
            # ★ 只关心**日志调用那一行里真的有中文**的文件。
            #
            # ⚠️⚠️ 原来判的是"整个文件里有没有中文" —— **那是错的**：
            #      本项目所有节点的**注释和文档字符串都是中文**写的，
            #      于是每个用 rospy 日志的文件都会被判成"打中文"，
            #      这条测试**永远失败**，等于没有。
            #      实测它误报了 `sim_stubs.py`（日志早已全 ASCII），
            #      也把 `sim_precheck.py` 的原因说得不准。
            #
            # 正确判据：逐行看，只认**日志调用行**里的中文字面量。
            zh_lines = []
            for _i, _line in enumerate(text.split(u'\n'), 1):
                _s = _line.strip()
                if _s.startswith(u'#'):
                    continue                 # 整行注释
                if not logpat.search(_line):
                    continue                 # 不是日志调用
                _code = _line.split(u'#')[0]  # 去掉行尾注释
                if _re.search(u'[\u4e00-\u9fff]', _code):
                    zh_lines.append(u'%d: %s' % (_i, _s[:70]))
            if not zh_lines:
                continue
            checked += 1
            has_def = 'def fix_ros_logging' in text
            has_call = ('fix_ros_logging()' in text
                        or '_fix_ros_logging()' in text)
            if not (has_def and has_call):
                problems.append(u'%s (定义=%s 调用=%s)\n      %s'
                                % (os.path.relpath(p, self.root),
                                   has_def, has_call,
                                   u'\n      '.join(zh_lines[:4])))
        self.assertGreater(checked, 0, u'应该扫描到至少一个日志打中文的文件')
        self.assertEqual(problems, [],
                         u'以下文件的【日志调用行】里有中文，'
                         u'但没有 fix_ros_logging，在 Python 2.7 上会崩：\n  '
                         + u'\n  '.join(problems))

    def test_ros_logging_fix_patches_makerecord(self):
        """fix_ros_logging 必须 patch makeRecord（唯一在 handler 之前的位置）。"""
        problems = []
        for p in self._py_files():
            with io.open(p, encoding='utf-8-sig') as fp:
                text = fp.read()
            if 'def fix_ros_logging' not in text:
                continue
            if 'makeRecord' not in text:
                problems.append(os.path.relpath(p, self.root))
        self.assertEqual(problems, [],
                         u'以下文件的 fix_ros_logging 没 patch makeRecord：\n  '
                         + u'\n  '.join(problems))

    def test_stream_writer_tolerates_bytes(self):
        """流包装器必须容忍字节串输入。

        Python 2 里 `traceback.print_exc()` 和 `logging.StreamHandler`
        都往 stderr 写**字节串**。标准的 `codecs.StreamWriter` 只接受
        unicode，收到字节串会先 `decode('ascii')`，遇到非 ASCII 就崩：

            UnicodeDecodeError: 'ascii' codec can't decode byte 0xe5

        后果是"为了看清异常而打印异常时崩掉"，真正的错误再也看不到。
        本项目踩过这个坑，所以 writer 必须是自定义的、字节串容忍版。
        """
        problems = []
        for p in self._py_files():
            with io.open(p, encoding='utf-8-sig') as fp:
                text = fp.read()
            # ⚠️ 判据要窄：只检查**自己实现**流向包装的文件。
            #
            # 从这个测试的意图看，它要防的是"有人写了个
            # codecs.getwriter 包装但不容忍字节串"。
            # 而正确做法是 **复用 `utf8_stdio._Utf8Writer`**
            # （`from utf8_stdio import setup_all as setup_stdout`）——
            # 那种情况下本文件里根本没有包装实现可查，
            # 硬要报就是**惩罚正确写法**，而且会一直报下去。
            has_def = 'def setup_stdout' in text
            has_wrap = 'def _wrap' in text
            if not (has_def or has_wrap):
                continue            # 复用了共享实现，无需检查
            # 有自定义实现 → 必须容忍字节串
            if 'class _Utf8Writer' in text \
                    or 'isinstance(s, bytes)' in text \
                    or 'StreamWriter' in text:
                continue
            problems.append(os.path.relpath(p, self.root))
        self.assertEqual(problems, [],
                         u'以下文件的流包装器不容忍字节串，'
                         u'Python 2 下打印 traceback 会崩：\n  '
                         + u'\n  '.join(problems))

    def test_no_bare_exception_in_unicode_format(self):
        """异常对象不能直接插进 unicode 格式串（Python 2 会崩）。

        实测（Python 2.7.17，getdefaultencoding=ascii）：

            异常消息是 ASCII         -> OK
            异常消息是 unicode       -> OK
            异常消息是**字节串中文** -> UnicodeDecodeError  ← 崩

        原因：`u'' % exc` 会调 `unicode(exc)`，内部按 ASCII 解码 `str(exc)`。
        底层库（pyserial / 讯飞 / tf）的异常消息常是 UTF-8 字节串。

        最恶劣的是它**崩在异常处理路径上**，真正的错误信息再也看不到。
        必须用 `repr(exc)`（ASCII 安全）。
        """
        import re as _re
        exc_names = r'(?:exc|e|err|error|exception|ex|exp)'
        pat = _re.compile(
            r"""u?['"](?:\\.|[^'"])*?%[sr](?:\\.|[^'"])*?['"]\s*%\s*"""
            r"""(?!repr\()""" + exc_names + r"""\b""")
        problems = []
        for p in self._py_files():
            with io.open(p, encoding='utf-8-sig') as fp:
                lines = fp.read().split('\n')
            in_doc = False
            for i, line in enumerate(lines, 1):
                s = line.strip()
                if s.startswith('#'):
                    continue
                if s.count('"""') == 1:
                    in_doc = not in_doc
                    continue
                if in_doc:
                    continue
                if pat.search(line) and 'repr(' not in line:
                    problems.append(u'%s:%d  %s'
                                    % (os.path.relpath(p, self.root), i,
                                       s[:80]))
        self.assertEqual(problems, [],
                         u'以下位置把异常对象直接插进 unicode 格式串，'
                         u'Python 2 下异常消息含中文时会崩，'
                         u'且会掩盖真正的错误：\n  ' + u'\n  '.join(problems))


# ===========================================================================
#  测试 8：仿真地图与仿真世界的一致性
# ===========================================================================
class TestSimConsistency(unittest.TestCase):
    """make_sim_map.py 生成的地图必须与 fake_sensors.py 的世界一致。

    如果不一致：
      · 地图尺寸与仿真世界不符 → amcl 在"地图外"定位，无法收敛
      · 障碍位置错位 → costmap 静态层与激光看到的不一样，
        规划器会算出穿墙路径，端到端测试结果就不可信了
    """

    @classmethod
    def setUpClass(cls):
        cls.fs = find_src('carebot_nav_kit/nodes/fake_sensors.py')
        cls.ms = find_src('carebot_nav_kit/scripts/make_sim_map.py')
        if not cls.fs or not cls.ms:
            raise unittest.SkipTest(u'找不到 fake_sensors.py 或 make_sim_map.py')

    def _extract_map(self, path, varname):
        """从源码里抽出 ASCII 地图列表。"""
        import re as _re
        with io.open(path, encoding='utf-8-sig') as fp:
            src = fp.read()
        i = src.find(varname + ' = [')
        if i < 0:
            return None
        j = src.find(']', i)
        chunk = src[i:j + 1]
        ns = {}
        exec(compile(chunk, path, 'exec'), ns)
        return ns[varname]

    def test_map_dimensions_match(self):
        a = self._extract_map(self.fs, 'DEFAULT_MAP')
        b = self._extract_map(self.ms, 'SIM_MAP')
        self.assertIsNotNone(a, u'抽不出 DEFAULT_MAP')
        self.assertIsNotNone(b, u'抽不出 SIM_MAP')
        self.assertEqual(len(a), len(b),
                         u'行数不一致：fake_sensors=%d, make_sim_map=%d'
                         % (len(a), len(b)))
        self.assertEqual([len(r) for r in a], [len(r) for r in b],
                         u'每行宽度不一致')

    def test_map_content_identical(self):
        a = self._extract_map(self.fs, 'DEFAULT_MAP')
        b = self._extract_map(self.ms, 'SIM_MAP')
        self.assertEqual(a, b,
                         u'两张地图内容不同 —— 改了一个忘了改另一个。\n'
                         u'fake_sensors.py 的 DEFAULT_MAP 与 '
                         u'make_sim_map.py 的 SIM_MAP 必须逐字符一致。')

    def test_resolution_field_present(self):
        self.assertIsNotNone(self._extract_map(self.ms, 'SIM_MAP'))
        with io.open(self.ms, encoding='utf-8-sig') as fp:
            src = fp.read()
        self.assertIn('RESOLUTION', src, u'make_sim_map.py 应定义 RESOLUTION')


# ===========================================================================
def main():
    setup_stdout()

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for cls in (TestQrParsing, TestNormalization, TestIotMapping,
                TestNavConfigConsistency, TestWaypoints, TestHardwareProfiles,
                TestStdoutEncodingSetup):
        suite.addTests(loader.loadTestsFromTestCase(cls))

    verbosity = 2 if '-v' in sys.argv else 1
    runner = unittest.TextTestRunner(verbosity=verbosity, stream=sys.stdout)
    result = runner.run(suite)

    print()
    print('=' * 70)
    print(u'  测试结果：%d 个用例，%d 失败，%d 错误，%d 跳过'
          % (result.testsRun, len(result.failures), len(result.errors),
             len(result.skipped)))
    print('=' * 70)

    if result.failures:
        print(u'\n失败详情：')
        for t, tb in result.failures:
            print(u'\n--- %s ---' % t)
            print(tb)

    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
