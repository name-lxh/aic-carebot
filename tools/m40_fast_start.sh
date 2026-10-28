#!/bin/bash
# ============================================================================
#  m40_fast_start.sh —— 30 秒内把全栈拉起来（并行启动 + 就绪即走）
# ============================================================================
#  和 m37 的区别：
#    · m37 是"顺序起 + 固定 sleep"，全栈要 2~3 分钟
#    · 本脚本**全部并行**，用 /initialpose + 话题探测判定就绪，不等固定时间
#    · 目标：脚本开始 -> 全栈就绪 <= 30 s，然后（可选）立刻开跑全流程
#
#  用法：
#    bash m40_fast_start.sh              # 只起栈，打印就绪耗时
#    bash m40_fast_start.sh --mission    # 起栈 + 就绪后自动后台跑 m36 全流程
#
#  ⚠️ 本文件里的 pkill 模式都带方括号（如 [s]tm32），
#     避免脚本自己的命令行被 pkill -f 匹配到（这个坑踩过 3 次）。
# ============================================================================
set -u

T0=$(date +%s)
LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
P=$WS/src/carebot_nav_kit/config/nav_v4
MAP=$WS/src/carebot_nav_kit/maps/arena_shared.yaml
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
export PYTHONIOENCODING=utf-8
source "$LXH/env.sh" 2>/dev/null

MISSION=0
[ "${1:-}" = "--mission" ] && MISSION=1

echo "════════ 1. 清理残留 ════════"
for p in slam_gmapping amcl move_base map_server laser_filter \
         lslidar_driver_node art_imu_01_node art_imu_node stm32_node; do
    pkill -x "$p" 2>/dev/null
done
# stm32.py / laser_filter are python scripts: process name is "python",
# so pkill -x cannot kill them -> duplicates -> two drivers fighting for /cmd_vel
pkill -f '[s]tm32.py' 2>/dev/null
pkill -f '[l]aser_filter' 2>/dev/null
pkill -f '[o]dom_imu.py' 2>/dev/null
pkill -f '[s]tatic_transform_publisher' 2>/dev/null
pkill -f '[o]dom_imu.py' 2>/dev/null
pkill -f '[q]r_detector.py' 2>/dev/null
pkill -f '[f]ace_gate.py' 2>/dev/null
pkill -f '[i]ot_bridge.py' 2>/dev/null
pkill -f '[t]ts_net' 2>/dev/null
# ★ astra 必须一起清：两个 astra launch 会抢同一个 USB 设备，
#   后起的那个只报 "Device open failed"，相机就永远没图（2026-10-21 踩到）
pkill -f '[a]stra.launch' 2>/dev/null
pkill -f '[c]amera_nodelet_manager' 2>/dev/null
sleep 1
# 设备释放需要一点时间，否则新实例还是 open failed
for i in 1 2 3 4 5 6; do
    pgrep -f '[c]amera_nodelet_manager' >/dev/null 2>&1 || break
    sleep 0.5
done
echo "  完成 ($(($(date +%s) - T0)) s)"

echo "════════ 2. roscore ════════"
# If another team's master owns 11311, our nodes would join it and fight
# their map_server/amcl (two TF sources). Kill it and start our own.
if pgrep -f 'bin/[r]osmaster' >/dev/null 2>&1; then
    pkill -f 'bin/[r]oslaunch' 2>/dev/null
    sleep 1
    pkill -9 -f 'bin/[r]osmaster' 2>/dev/null
    pkill -9 -f 'bin/[r]oscore' 2>/dev/null
    sleep 1
fi
if ! ss -ltn 2>/dev/null | grep -q ':11399'; then
    setsid nohup roscore -p 11399 > /tmp/f_roscore.log 2>&1 < /dev/null &
    for i in $(seq 1 50); do
        ss -ltn 2>/dev/null | grep -q ':11399' && break
        sleep 0.2
    done
fi
ss -ltn 2>/dev/null | grep -q ':11399' && echo "  OK ($(($(date +%s) - T0)) s)" \
    || { echo "  FAIL"; tail -5 /tmp/f_roscore.log; exit 1; }

echo "════════ 3. 并行起全部节点 ════════"
setsid nohup roslaunch lslidar_driver lslidar_serial.launch > /tmp/f_lidar.log 2>&1 < /dev/null &
setsid nohup roslaunch base_control stm32.launch           > /tmp/f_base.log  2>&1 < /dev/null &
setsid nohup roslaunch "$WS/src/carebot_nav_kit/launch/include/imu.launch" \
                                                          > /tmp/f_imu.log   2>&1 < /dev/null &
setsid nohup rosrun tf static_transform_publisher 0 0 0.06 0 0 0 base_footprint base_link 50 \
                                                          > /dev/null 2>&1 < /dev/null &
setsid nohup rosrun tf static_transform_publisher 0 0 0.21 0 0 0 base_link laser_link 50 \
                                                          > /dev/null 2>&1 < /dev/null &
setsid nohup rosrun tf static_transform_publisher 0.125 0 0.11 0 0 0 base_link IMU_link 50 \
                                                          > /dev/null 2>&1 < /dev/null &
setsid nohup python "$LXH/_tools/odom_imu.py"             > /tmp/f_odom.log  2>&1 < /dev/null &
setsid nohup roslaunch "$WS/src/carebot_nav_kit/launch/include/laser_filter.launch" \
                                                          > /tmp/f_filter.log 2>&1 < /dev/null &
setsid nohup rosrun map_server map_server "$MAP" __name:=map_server \
                                                          > /tmp/f_map.log   2>&1 < /dev/null &
# 相机要在子 shell 里 source 厂商环境（会覆盖 ROS_PACKAGE_PATH）
( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
  setsid nohup roslaunch astra_launch astra.launch > /tmp/f_astra.log 2>&1 < /dev/null & )

# 导航参数（rosparam 立即生效，不用等）
rosparam load "$P/amcl_params.yaml" /amcl
rosparam delete /move_base 2>/dev/null
rosparam load "$P/costmap_common_params_global.yaml" /move_base/global_costmap
rosparam load "$P/costmap_common_params_local.yaml"  /move_base/local_costmap
rosparam load "$P/global_costmap_params.yaml" /move_base
rosparam load "$P/local_costmap_params.yaml"  /move_base
rosparam load "$P/global_planner_params.yaml" /move_base
rosparam load "$P/dwa_local_planner_params.yaml" /move_base
rosparam load "$P/recovery_params.yaml" /move_base
rosparam set /move_base/base_global_planner global_planner/GlobalPlanner
rosparam set /move_base/base_local_planner  dwa_local_planner/DWAPlannerROS
rosparam set /move_base/global_costmap/robot_base_frame base_footprint
rosparam set /move_base/local_costmap/robot_base_frame  base_footprint

setsid nohup rosrun amcl amcl __name:=amcl scan:=/scan_filtered odom:=/odom_clean \
                                                          > /tmp/f_amcl.log  2>&1 < /dev/null &
setsid nohup rosrun move_base move_base __name:=move_base scan:=/scan_filtered \
                                                          > /tmp/f_mb.log    2>&1 < /dev/null &

# ★ 2026-10-23：速度看门狗（底盘会锁存最后速度，没人发指令时必须补零）
( setsid nohup python2 -u "$LXH/_tools/vel_watchdog.py" > /tmp/f_wd.log 2>&1 < /dev/null & )

# 业务节点（相机起来后自动连上）
setsid nohup roslaunch carebot_qr qr_detect.launch        > /tmp/f_qr.log    2>&1 < /dev/null &
setsid nohup roslaunch carebot_face face.launch           > /tmp/f_face.log  2>&1 < /dev/null &
setsid nohup roslaunch carebot_iot iot.launch token_file:=$LXH/.ha_token \
                                                          > /tmp/f_iot.log   2>&1 < /dev/null &
# ★ 2026-10-23：改为离线播报（WAV 查表，不联网）；tts_net.py 保留作回退
#   TTS_OFFLINE=0 可退回在线 TTS
if [ "${TTS_OFFLINE:-1}" = "0" ]; then
    ( setsid nohup python2 -u "$LXH/_tools/tts_net.py" > /tmp/f_tts.log 2>&1 < /dev/null & )
else
    ( setsid nohup python2 -u "$LXH/_tools/tts_play.py" > /tmp/f_tts.log 2>&1 < /dev/null & )
fi
echo "  已全部拉起 ($(($(date +%s) - T0)) s)"

echo "════════ 4. 等就绪（发初始位姿 + TF + 服务）════════"
python2 "$LXH/_tools/m40_wait_ready.py" --deadline 24
RC=$?

# ★ 底盘驱动在线检查：2026-10-20 就是 stm32_node 死了，车不动、开环 odom 却一直涨，
#   上层完全"以为"自己在走。这里用 /cmd_vel 的订阅者做静态判定（不发指令、车不动）。
echo "════════ 4.5 底盘驱动检查 ════════"
base_ok() { rostopic info /cmd_vel 2>/dev/null | grep -q '/stm32_node'; }
if base_ok; then
    echo "  ✅ /stm32_node 已订阅 /cmd_vel"
else
    echo "  ⚠ 没检测到 /stm32_node，重启一次底盘驱动"
    pkill -x stm32_node 2>/dev/null; sleep 1
    setsid nohup roslaunch base_control stm32.launch > /tmp/f_base2.log 2>&1 < /dev/null &
    for i in $(seq 1 20); do base_ok && break; sleep 0.4; done
    if base_ok; then echo "  ✅ 重启后正常"; else echo "  ❌ 底盘驱动仍不在线（车不会动！查串口/急停）"; fi
fi

EL=$(( $(date +%s) - T0 ))
echo ""
echo "════════ 5. 结果 ════════"
echo "  全栈启动总耗时: ${EL} s   （目标 <= 30 s）"
if [ "$RC" != "0" ]; then
    echo "  [warn] 有缺项，检查 /tmp/f_*.log"
fi

# ★ 2026-10-23：相机自动曝光要爬十几秒，画面没稳定前二维码/人脸都容易读不到。
#   发车之前等它稳定（最多 15 s，稳定就提前走），不阻塞调试用的起栈。
if [ "$MISSION" = "1" ]; then
    echo "════════ 4.6 相机预热 ════════"
    python2 "$LXH/_tools/camera_warmup.py" --timeout 15 2>&1 | tail -3
fi

if [ "$MISSION" = "1" ]; then
    echo "════════ 6. 直接开跑全流程 ════════"
    rm -f /tmp/m36_run7.log
    ( setsid nohup python2 -u "$LXH/_tools/m36_mission_v5.py" \
        > /tmp/m36_run7.log 2>&1 < /dev/null & )
    sleep 3
    echo "  mission pid=$(pgrep -f 'm36_mission_[v]5' | head -1)  日志 /tmp/m36_run7.log"
fi
exit 0
