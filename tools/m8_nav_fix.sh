#!/bin/bash
# ============================================================================
#  m8_nav_fix.sh —— 修导航参数与残留节点
# ============================================================================
#  发现的问题：
#    1. `robot_radius` 仍是 **0.18**，而这台车是 50 cm 直径（半径 0.25）。
#       之前只改了本地文件，**没有上传到车上**，所以 rosparam 读到的还是旧值。
#       -> 半径偏小会让规划器贴着墙走、代价地图不把车体算进去，很危险。
#    2. 有 **两套静态 TF**（m2 起了 3 个、m3 又起了 3 个），同一对 frame
#       两个发布者，rviz/tf 会抖。
#    3. `odom_relay` 还在跑（把 /odometry/filtered 转 /odom），
#       和我们的 odom_imu 冲突。
#    4. `/slam_gmapping` 还在节点表里（可能有残留注册）。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
NAV=$WS/src/carebot_nav_kit
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 1. 清理重复节点 ════════"
# 只保留一套静态 TF：全杀，然后重起 3 个
pkill -f 'static_transform_publisher' 2>/dev/null
pkill -f '[o]dom_relay' 2>/dev/null
sleep 3
echo "  静态TF 残留: $(pgrep -fc 'static_transform_publisher' 2>/dev/null || echo 0)"
echo "  odom_relay : $(pgrep -fc '[o]dom_relay' 2>/dev/null || echo 0)"

setsid nohup rosrun tf static_transform_publisher 0 0 0.06 0 0 0 base_footprint base_link 50 \
    > /dev/null 2>&1 < /dev/null &
sleep 1
setsid nohup rosrun tf static_transform_publisher 0 0 0.21 0 0 0 base_link laser_link 50 \
    > /dev/null 2>&1 < /dev/null &
sleep 1
setsid nohup rosrun tf static_transform_publisher 0.125 0 0.11 0 0 0 base_link IMU_link 50 \
    > /dev/null 2>&1 < /dev/null &
sleep 2
echo "  已重起 3 个静态 TF（一套）"

echo ""
echo "════════ 2. 停 move_base，修参数后重起 ════════"
pkill -x move_base 2>/dev/null
sleep 3

rosparam load "$NAV/config/nav/costmap_common_params.yaml"  /move_base/global_costmap 2>/dev/null
rosparam load "$NAV/config/nav/global_costmap_params.yaml"  /move_base/global_costmap 2>/dev/null
rosparam load "$NAV/config/nav/costmap_common_params.yaml"  /move_base/local_costmap 2>/dev/null
rosparam load "$NAV/config/nav/local_costmap_params.yaml"   /move_base/local_costmap 2>/dev/null
rosparam load "$NAV/config/nav/global_planner_params.yaml"  /move_base 2>/dev/null
rosparam load "$NAV/config/nav/dwa_local_planner_params.yaml" /move_base 2>/dev/null
rosparam set /move_base/base_global_planner global_planner/GlobalPlanner
rosparam set /move_base/base_local_planner  dwa_local_planner/DWAPlannerROS

GR=$(rosparam get /move_base/global_costmap/robot_radius 2>/dev/null)
LR=$(rosparam get /move_base/local_costmap/robot_radius 2>/dev/null)
echo "  global robot_radius = $GR"
echo "  local  robot_radius = $LR"
if [ "$GR" != "0.25" ]; then
    echo "  [!] 参数仍是 $GR —— 说明车上的 costmap_common_params.yaml 还是旧版"
    echo "      我会直接用 rosparam set 强制覆盖"
    rosparam set /move_base/global_costmap/robot_radius 0.25
    rosparam set /move_base/local_costmap/robot_radius  0.25
    echo "      已强制设为 0.25"
fi

# 局部规划限速（保守一点，别冲）
rosparam get /move_base/DWAPlannerROS/max_vel_x 2>/dev/null | sed 's/^/  DWA max_vel_x = /'
rosparam set /move_base/DWAPlannerROS/max_vel_x 0.20
rosparam set /move_base/DWAPlannerROS/max_vel_theta 0.6
echo "  已限制: max_vel_x=0.20  max_vel_theta=0.6"

setsid nohup rosrun move_base move_base __name:=move_base scan:=/scan_filtered \
    > /tmp/m8_movebase.log 2>&1 < /dev/null &
sleep 22
echo ""
echo "  move_base 进程: $(pgrep -xc move_base 2>/dev/null || echo 0)"
echo "  代价地图:"
for t in /move_base/global_costmap/costmap /move_base/local_costmap/costmap; do
    printf '    %-44s ' "$t"
    timeout 8 rostopic list 2>/dev/null | grep -qx "$t" && echo OK || echo '--'
done
echo "  错误:"
grep -iE 'error|ERROR|died|Failed|lethal' /tmp/m8_movebase.log 2>/dev/null | head -6 | sed 's/^/    /'

echo ""
echo "════════ 3. 确认 radius 生效（从参数服务器读）════════"
echo "  global: $(rosparam get /move_base/global_costmap/robot_radius 2>/dev/null)"
echo "  local : $(rosparam get /move_base/local_costmap/robot_radius 2>/dev/null)"

echo ""
echo "════════ 4. TF 链 ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo '--'
done

echo ""
echo "════════ 完成 ════════"
