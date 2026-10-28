#!/bin/bash
# ============================================================================
#  m37_clean_start.sh —— 干净启动：先杀掉所有会干扰的进程，再起传感器栈 + 导航栈
# ============================================================================
#  为什么需要
#    · 车上同时跑着多套栈（tianxing_team 的 ekf+gmapping、别人的遥控 teleop 等），
#      两套栈同时发 map->odom / 抢 /cmd_vel，定位和规划会互相污染，
#      症状是 "Failed to get a plan" 刷屏、DWA 无解、底盘收到打架的速度指令。
#    · 我们自己的 amcl/move_base 如果挂在旧 master 上，会变成僵尸节点。
#
#  它做什么
#    0. --dry-run 只打印将要杀谁
#    1. 杀 roslaunch（先杀 launch，避免它把节点重新拉起来）
#    2. 杀 roscore/rosmaster（保证 11311 是干净的、属于我们的）
#    3. 杀导航/建图/遥控/rviz 节点，以及我们自己的残留
#    4. 起 roscore -p 11399
#    5. m14_up.sh（雷达/底盘/IMU/TF/odom/滤波）
#    6. m34_nav_v4.sh（map_server + amcl + move_base，V3 参数）
#
#  用法
#    bash m37_clean_start.sh --dry-run     # 只看会杀谁
#    bash m37_clean_start.sh               # 真杀 + 起栈
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
TOOLS=$LXH/_tools
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213

DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

kill_f() {   # kill_f <模式> <说明>
    local pat="$1" desc="$2"
    local pids
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -v "^$$\$" || true)
    if [ -z "$pids" ]; then
        return 0
    fi
    echo "    [$desc] $pat -> $(echo $pids | tr '\n' ' ')"
    [ "$DRY" = "1" ] || kill $pids 2>/dev/null
}

echo "════════ 0. 预演 = $DRY ════════"
echo "  当前 11311 上的节点数: $(timeout 6 rosnode list 2>/dev/null | grep -c . || echo 0)"

echo ""
echo "════════ 1. 杀掉别人的 launch（先断根）════════"
kill_f 'bin/roslaunch' 'roslaunch'
[ "$DRY" = "1" ] || sleep 3

echo ""
echo "════════ 2. 杀掉 roscore/rosmaster（保证 11311 干净）════════"
kill_f 'bin/rosmaster' 'master'
kill_f 'bin/roscore' 'core'
[ "$DRY" = "1" ] || sleep 3

echo ""
echo "════════ 3. 杀掉会干扰的节点/进程 ════════"
kill_f 'ekf_localization_node' 'EKF'
kill_f 'slam_gmapping' 'gmapping'
kill_f 'simple_gmapping' 'gmapping2'
kill_f 'move_base' 'move_base'
kill_f 'bin/amcl' 'amcl'
kill_f 'bin/map_server' 'map_server'
kill_f 'robot_state_publisher' 'robot_state_pub'
kill_f 'state_publisher' 'state_pub'
kill_f 'joint_state_publisher' 'joint_state_pub'
kill_f 'teleop_twist_keyboard' 'teleop'
kill_f 'bin/rviz' 'rviz'
kill_f 'ld_navigation' 'their-nav'
kill_f 'driver_runtime' 'their-driver'
kill_f 'sd_navigation' 'their-nav2'
[ "$DRY" = "1" ] || sleep 2

echo ""
echo "  剩余 ROS 相关进程："
ps aux | grep -E '[r]osmaster|[r]oscore|[r]oslaunch|[m]ove_base|[a]mcl|[m]ap_server|[g]mapping|[e]kf_localization' \
    | awk '{print "    " $2, $11, $12, $13}' | head -8
echo "  （空 = 干净）"

if [ "$DRY" = "1" ]; then
    echo ""
    echo "  预演结束，没有杀任何东西。"
    exit 0
fi

echo ""
echo "════════ 4. 起干净的 master（11311）════════"
source "$LXH/env.sh" 2>/dev/null
if pgrep -f 'bin/rosmaster' >/dev/null 2>&1; then
    echo "  [warn] 还有 rosmaster 没杀掉，先跳过启动"
else
    setsid nohup roscore -p 11399 > /tmp/m37_roscore.log 2>&1 < /dev/null &
    sleep 9
fi
if (command -v ss >/dev/null && ss -ltn | grep -q ':11399') || netstat -ltn 2>/dev/null | grep -q ':11399'; then
    echo "  OK 11311 在监听"
else
    echo "  [FAIL] 11311 没起来，看 /tmp/m37_roscore.log"
    tail -5 /tmp/m37_roscore.log 2>/dev/null
    exit 1
fi

echo ""
echo "════════ 5. 传感器栈（m14）════════"
bash "$TOOLS/m14_up.sh" 2>&1 | tail -8

echo ""
echo "════════ 6. 导航栈（m34，V3 参数）════════"
bash "$TOOLS/m34_nav_v4.sh" 2>&1 | tail -10

echo ""
echo "════════ 7. 业务节点（IOT 桥接 + 在线 TTS）════════"
# iot_bridge（带令牌）+ 在线 TTS：设备控制/环境读数/播报都靠它们
for p in $(pgrep -f 'tts_net'); do kill "$p" 2>/dev/null; done
sleep 1
setsid nohup roslaunch carebot_iot iot.launch token_file:=/home/ub/Hajimi/LXH/.ha_token     > /tmp/m37_iot.log 2>&1 < /dev/null &
sleep 6
( setsid nohup python2 -u "$TOOLS/tts_net.py" > /tmp/m37_tts.log 2>&1 < /dev/null & )
sleep 3
echo "  iot_bridge: $(pgrep -f '[i]ot_bridge.py' | head -1)"
echo "  tts_net   : $(pgrep -f 'tts_net' | head -1)"
timeout 8 rosservice list 2>/dev/null | grep -E '^/iot/(refresh|control)' | sed 's/^/  /'

echo ""
echo "════════ 8. 下一步 ════════"
echo "  1) 单帧匹配定位 -> set_init_pose.py X Y YAW"
echo "  2) m28_preflight.py  必须 ✅"
echo "  3) m36_mission_v5.py --fake-face"
