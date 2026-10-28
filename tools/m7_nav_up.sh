#!/bin/bash
# ============================================================================
#  m7_nav_up.sh —— 起导航栈（map_server + amcl + move_base），不含 bringup
# ============================================================================
#  ⚠️ 为什么不直接用 navigation.launch
#
#  `navigation.launch` 第 50 行 include 了 `bringup.launch`，
#  而 bringup 会再起一遍 **lidar / 底盘 / IMU / EKF / 静态TF**。
#  这些东西 m1/m3/m6 已经起来并在正常工作，再起一遍会：
#    · 两个 lidar 驱动抢同一个 `/dev/carebot_laser_ls`
#    · 两套静态 TF 同时发布同一个 base→laser，rviz 抖动、建图/定位扭曲
#    · EKF 再发一份 /odometry/filtered 并 relay 到 /odom，和 odom_imu 打架
#
#  所以这里**只起导航需要的三件**：
#      map_server（加载干净地图）+ amcl（定位）+ move_base（规划）
#
#  ⚠️ 还必须**先停 gmapping**：gmapping 和 amcl 都会发布 map→odom，
#     同时存在会让 TF 树彻底乱掉。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
NAV=$WS/src/carebot_nav_kit
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

MAP=${1:-$NAV/maps/arena_clean.yaml}

echo "════════ 0. 参数 ════════"
echo "  地图: $MAP"
[ -f "$MAP" ] && echo "    OK 存在" || { echo "    FAIL 不存在"; exit 1; }

echo ""
echo "════════ 1. 停 gmapping（避免和 amcl 抢 map→odom）════════"
pkill -x slam_gmapping 2>/dev/null
sleep 3
echo "  gmapping=$(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. 确认基础件仍在 ════════"
for t in /scan /scan_filtered /imu_data /odom_clean; do
    printf '  %-16s ' "$t"
    hz=$(timeout 8 rostopic hz "$t" 2>&1 | grep -m1 average || echo '')
    echo "${hz:-★无数据}"
done
printf '  %-16s ' "odom->base TF"
timeout 5 rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -q Translation && echo OK || echo FAIL

echo ""
echo "════════ 3. 停掉可能残留的旧导航节点 ════════"
pkill -x amcl 2>/dev/null
pkill -x move_base 2>/dev/null
pkill -x map_server 2>/dev/null
sleep 2
echo "  已清理"

echo ""
echo "════════ 4. map_server ════════"
setsid nohup rosrun map_server map_server "$MAP" \
    __name:=map_server > /tmp/m7_map.log 2>&1 < /dev/null &
sleep 8
if timeout 10 rostopic list 2>/dev/null | grep -qx /map; then
    echo "  OK /map 已发布"
    head -3 /tmp/m7_map.log | sed 's/^/    /'
else
    echo "  FAIL /map 没有"
    cat /tmp/m7_map.log | sed 's/^/    /'
    exit 1
fi

echo ""
echo "════════ 5. amcl ════════"
rosparam load "$NAV/config/nav/amcl_params.yaml" /amcl 2>/dev/null
setsid nohup rosrun amcl amcl __name:=amcl scan:=/scan_filtered odom:=/odom_clean \
    > /tmp/m7_amcl.log 2>&1 < /dev/null &
sleep 10
echo "  amcl 进程: $(pgrep -xc amcl 2>/dev/null || echo 0)"
grep -iE 'error|ERROR|die' /tmp/m7_amcl.log 2>/dev/null | head -3 | sed 's/^/    /'

echo ""
echo "════════ 6. move_base ════════"
MB=move_base
# 依次把参数塞到 /move_base 命名空间
rosparam load "$NAV/config/nav/costmap_common_params.yaml"  /move_base/global_costmap 2>/dev/null
rosparam load "$NAV/config/nav/global_costmap_params.yaml"  /move_base/global_costmap 2>/dev/null
rosparam load "$NAV/config/nav/costmap_common_params.yaml"  /move_base/local_costmap 2>/dev/null
rosparam load "$NAV/config/nav/local_costmap_params.yaml"   /move_base/local_costmap 2>/dev/null
rosparam load "$NAV/config/nav/global_planner_params.yaml"  /move_base 2>/dev/null
rosparam load "$NAV/config/nav/dwa_local_planner_params.yaml" /move_base 2>/dev/null
rosparam set /move_base/base_global_planner global_planner/GlobalPlanner
rosparam set /move_base/base_local_planner  dwa_local_planner/DWAPlannerROS
echo "  robot_radius=$(rosparam get /move_base/global_costmap/robot_radius 2>/dev/null)"

setsid nohup rosrun move_base move_base __name:=move_base scan:=/scan_filtered \
    > /tmp/m7_movebase.log 2>&1 < /dev/null &
sleep 20
echo "  move_base 进程: $(pgrep -xc move_base 2>/dev/null || echo 0)"
echo "  关键话题:"
for t in /move_base/global_costmap/costmap /move_base/local_costmap/costmap /amcl_pose /move_base/status; do
    printf '    %-44s ' "$t"
    timeout 6 rostopic list 2>/dev/null | grep -qx "$t" && echo OK || echo '--'
done
grep -iE 'error|ERROR|died|Failed' /tmp/m7_movebase.log 2>/dev/null | head -5 | sed 's/^/    /'

echo ""
echo "════════ 7. TF 链 ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo '--'
done

echo ""
echo "════════ 8. 节点 ════════"
timeout 10 rosnode list 2>/dev/null | grep -vE 'tf_echo|_tf_echo' | sed 's/^/  /'

echo ""
echo "════════ 完成 ════════"
echo "  下一步：设初始位姿 + 发目标点"
