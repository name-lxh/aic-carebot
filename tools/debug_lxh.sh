#!/bin/bash
# ============================================================================
#  debug_lxh.sh —— LXH 包调试自检（在车上执行）
# ============================================================================
#  用户要求："部署完成后先调试再准备建图"
#  所以这一步**不建图、不动机器人**，只做静态/离线的自检：
#     1. 单元测试（58 个用例，Py2 必须全通过）
#     2. 12 个节点能否被找到
#     3. 4 个 launch 能否被 roslaunch 解析
#     4. 消息/服务能否 import
#     5. 纯逻辑模块能否 import
#  ⚠️ 全部只读，不发速度、不起 move_base。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
FAIL=0

echo "════════ 环境 ════════"
set +u
# ⚠️⚠️ 顺序很重要：**先把所有 setup.bash source 完，最后再手工补路径**。
#
# 踩过：一开始写的是「source → 补路径 → 再 source 我们的 devel」，
# 结果第二次 source 触发 catkin 的 unroll，**把刚补的路径又剥掉了**，
# 导致 ROS_PACKAGE_PATH 里没有 omniWheelCareRobot/rosCode/src，
# astra_launch 找不到，eldercare_full.launch 解析失败 —— 看着像"缺包"，
# 其实只是**环境没配全**。
source /opt/ros/kinetic/setup.bash
[ -f /home/ub/catkin_ws/devel/setup.bash ] && source /home/ub/catkin_ws/devel/setup.bash
source "$WS/devel/setup.bash"
# ★ 最后补路径（对抗 unroll），必须在所有 source 之后
for d in /home/ub/omniWheelCareRobot/rosCode/src /home/ub/catkin_ws/src; do
    [ -d "$d" ] || continue
    case ":${ROS_PACKAGE_PATH:-}:" in
        *":$d:"*) ;;
        *) ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$d" ;;
    esac
done
export ROS_PACKAGE_PATH
set -u
echo "  ROS_DISTRO=${ROS_DISTRO:-?}  Python: $(python --version 2>&1)"
echo "  ROS_PACKAGE_PATH:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep . | sed 's/^/      /'

echo ""
echo "════════ 1. ★ 纯逻辑单元测试 ════════"
T="$WS/src/carebot_nav_kit/test/test_pure_logic.py"
if [ -f "$T" ]; then
    OUT=$(cd "$(dirname "$T")" && timeout 300 python "$T" 2>&1)
    echo "$OUT" | grep -E '^(Ran |OK|FAILED|ERROR)' | sed 's/^/      /'
    echo "$OUT" | grep -E '测试结果|^OK$' | tail -2 | sed 's/^/      /'
    if echo "$OUT" | grep -qE '^OK$'; then
        echo "      [OK] 单元测试通过"
    else
        echo "      [FAIL] 单元测试有问题"
        echo "$OUT" | tail -25 | sed 's/^/        /'
        FAIL=$((FAIL+1))
    fi
else
    echo "      [!] 找不到 $T"
    FAIL=$((FAIL+1))
fi

echo ""
echo "════════ 2. 节点可发现性 ════════"
NODES="carebot_eldercare/mission_manager.py carebot_eldercare/sim_stubs.py \
carebot_face/face_gate.py carebot_iot/iot_bridge.py \
carebot_nav_kit/fake_sensors.py carebot_nav_kit/nav_check.py \
carebot_nav_kit/odom_monitor.py carebot_nav_kit/set_initial_pose.py \
carebot_nav_kit/sim_precheck.py carebot_nav_kit/waypoint_nav.py \
carebot_qr/qr_detector.py carebot_voice/voice_bridge.py"
ok=0; bad=0
for n in $NODES; do
    pkg=${n%%/*}; base=${n##*/}
    p=$(rospack find "$pkg" 2>/dev/null)
    if [ -z "$p" ]; then
        printf '      FAIL %-42s 包找不到\n' "$n"; bad=$((bad+1)); continue
    fi
    f=$(find "$p" -name "$base" -type f 2>/dev/null | head -1)
    if [ -n "$f" ] && [ -x "$f" ]; then
        ok=$((ok+1))
    elif [ -n "$f" ]; then
        printf '      WARN %-42s 存在但不可执行\n' "$n"; bad=$((bad+1))
    else
        printf '      FAIL %-42s 文件不存在\n' "$n"; bad=$((bad+1))
    fi
done
echo "      可执行 $ok / $((ok+bad))"
[ "$bad" -eq 0 ] || FAIL=$((FAIL+1))

echo ""
echo "════════ 3. launch 解析 ════════"
SETUP=$(mktemp -d)
cat > "$SETUP/x.launch" <<'LEOF'
<launch><node pkg="rosgraph" type="rosgraph" name="dummy_probe"/></launch>
LEOF
rm -rf "$SETUP"
for l in $(find "$LXH" -name '*.launch' 2>/dev/null | sort); do
    rel=${l#$LXH/}
    N=$(timeout 40 roslaunch --nodes "$l" 2>/tmp/_lr.err | wc -l)
    if [ "$N" -gt 0 ]; then
        printf '      OK   %-52s %d 节点\n' "$rel" "$N"
    else
        printf '      FAIL %-52s\n' "$rel"
        head -3 /tmp/_lr.err | sed 's/^/            /'
        FAIL=$((FAIL+1))
    fi
done

echo ""
echo "════════ 4. 消息 / 服务 import ════════"
python - <<'PYEOF' 2>&1 | sed 's/^/      /'
from __future__ import print_function
mods = [
 ('carebot_iot_msgs.srv','Control'),
 ('std_srvs.srv','Trigger'),
 ('move_base_msgs.msg','MoveBaseAction'),
 ('nav_msgs.msg','OccupancyGrid'),
 ('sensor_msgs.msg','LaserScan'),
 ('geometry_msgs.msg','PoseStamped'),
 ('actionlib',None), ('tf',None), ('yaml',None), ('serial',None),
 ('cv2',None), ('numpy',None),
]
bad = 0
for m, attr in mods:
    try:
        mod = __import__(m, fromlist=['*'])
        if attr: getattr(mod, attr)
        print('OK   %s%s' % (m, '.'+attr if attr else ''))
    except Exception as e:
        print('FAIL %s: %r' % (m, e)); bad += 1
print('%d/%d' % (len(mods)-bad, len(mods)))
raise SystemExit(1 if bad else 0)
PYEOF
[ $? -eq 0 ] || FAIL=$((FAIL+1))

echo ""
echo "════════ 5. 纯逻辑模块 import ════════"
for m in sim_arena ros_logfix utf8_stdio verify_y_mapping; do
    d=$(find "$LXH" -name "$m.py" 2>/dev/null | head -1)
    if [ -z "$d" ]; then printf '      SKIP %-16s 文件不在\n' "$m"; continue; fi
    dd=$(dirname "$d")
    ( cd "$dd" && python -c "import $m" 2>/dev/null ) \
        && printf '      OK   %s\n' "$m" || { printf '      FAIL %s\n' "$m"; FAIL=$((FAIL+1)); }
done

echo ""
echo "════════ 结论 ════════"
if [ $FAIL -eq 0 ]; then
    echo "  [OK] 全部自检通过"
else
    echo "  [FAIL] 有 $FAIL 项不通过"
fi
exit $FAIL
