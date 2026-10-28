#!/bin/bash
# ============================================================================
#  m24_slam_to_nav.sh —— 建图模式 -> 导航模式，并接续 gmapping 的真实位姿
# ============================================================================
#  ## 关键技巧
#  切换前 gmapping 正在跑，它通过 map->odom 精确知道机器人在图上的位置。
#  在停掉它**之前**读出该位姿，直接作为 AMCL 初值 —— 比 localize.py 盲搜可靠
#  （旧图上实测最佳/次优只差 1.5%，盲搜没法用）。
#
#  ## 路线"尽可能远离墙"
#    inflation_radius    0.36  比车半径 0.28 大 8cm，形成避让带
#    cost_scaling_factor 3.0   衰减慢 -> 靠墙代价快速升高 -> 规划偏向中间
#  （不用 0.45 那种大膨胀：会把窄通道封死，之前实测 0/4 全失败）
#
#  ⚠️ 本脚本**不再用 heredoc 写 Python**：Py2 下 heredoc 里的中文会触发
#     SyntaxError: Non-ASCII character ... but no encoding declared。
#     改为调用独立的 grab_pose.py / set_init_pose.py。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
NAV=$LXH/catkin_ws/src/carebot_nav_kit
MAP=$NAV/maps/arena_final.yaml
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"
rm -f /tmp/grabbed_pose.txt

echo "════════ 1. ★ 停 gmapping 之前先读真实位姿 ════════"
timeout 60 python "$LXH/_tools/grab_pose.py" 2>&1 | sed 's/^/  /'
if [ ! -f /tmp/grabbed_pose.txt ]; then
    echo "  [FAIL] 取不到位姿，中止（不用错误初值冒险）"
    exit 1
fi
read PX PY PYA < /tmp/grabbed_pose.txt
echo "  将用作 AMCL 初值: x=$PX y=$PY yaw=$PYA"

echo ""
echo "════════ 2. 停 gmapping，起 map_server + amcl ════════"
pkill -x slam_gmapping 2>/dev/null
sleep 4
echo "  gmapping=$(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"

setsid nohup rosrun map_server map_server "$MAP" __name:=map_server \
    > /tmp/m24_map.log 2>&1 < /dev/null &
sleep 8
timeout 10 rostopic list 2>/dev/null | grep -qx /map && echo "  map_server OK" \
    || { echo "  map_server FAIL"; cat /tmp/m24_map.log; exit 1; }

rosparam load "$NAV/config/nav/amcl_params.yaml" /amcl
setsid nohup rosrun amcl amcl __name:=amcl scan:=/scan_filtered odom:=/odom_clean \
    > /tmp/m24_amcl.log 2>&1 < /dev/null &
sleep 10
echo "  amcl=$(pgrep -xc amcl 2>/dev/null || echo 0)"

echo ""
echo "════════ 3. 起 move_base（远离墙的参数）════════"
rosparam delete /move_base 2>/dev/null
sleep 1
rosparam load "$NAV/config/nav/costmap_common_params.yaml" /move_base/global_costmap
rosparam load "$NAV/config/nav/costmap_common_params.yaml" /move_base/local_costmap
rosparam load "$NAV/config/nav/global_costmap_params.yaml" /move_base
rosparam load "$NAV/config/nav/local_costmap_params.yaml"  /move_base
rosparam load "$NAV/config/nav/global_planner_params.yaml" /move_base
rosparam load "$NAV/config/nav/dwa_local_planner_params.yaml" /move_base
rosparam set /move_base/base_global_planner global_planner/GlobalPlanner
rosparam set /move_base/base_local_planner  dwa_local_planner/DWAPlannerROS
rosparam set /move_base/global_costmap/robot_radius 0.28
rosparam set /move_base/local_costmap/robot_radius  0.28
rosparam set /move_base/global_costmap/inflation_layer/inflation_radius 0.36
rosparam set /move_base/local_costmap/inflation_layer/inflation_radius  0.36
rosparam set /move_base/global_costmap/inflation_layer/cost_scaling_factor 3.0
rosparam set /move_base/local_costmap/inflation_layer/cost_scaling_factor  3.0
echo "  radius=0.28 inflation=$(rosparam get /move_base/global_costmap/inflation_layer/inflation_radius) cost_scaling=$(rosparam get /move_base/global_costmap/inflation_layer/cost_scaling_factor)"

setsid nohup rosrun move_base move_base __name:=move_base scan:=/scan_filtered \
    > /tmp/m24_mb.log 2>&1 < /dev/null &
sleep 25
printf '  /move_base/status: '
timeout 12 rostopic hz /move_base/status 2>&1 | grep -m1 average || echo '★无消息'

echo ""
echo "════════ 4. 设 AMCL 初始位姿（gmapping 真值）════════"
timeout 40 python "$LXH/_tools/set_init_pose.py" "$PX" "$PY" "$PYA" 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 5. TF 链 ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo '--'
done

echo ""
echo "  切换完成"
