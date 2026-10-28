#!/bin/bash
# ============================================================================
#  m26_nav_shared.sh —— 用「共享地图 + 其他组的导航参数」起导航栈
# ============================================================================
#  ## 为什么改用共享地图
#
#  车上三个组（edg_team / tianxing_team / laoyou）用的是**同一个地图文件**
#  `talos_map_1016_0808`（md5 完全相同）—— 那是大家公认可用的地图。
#  相比之下我自己重建的图虽然几何正确，但覆盖不全、坐标系也是新的，
#  航点全靠推算。用共享图 + 别人的航点，不确定性小得多。
#
#  ## 采用他们的参数（已拷到 config/nav_shared/）
#
#      robot_radius        0.18      （我之前用 0.28，偏大）
#      inflation_radius    0.50      （我之前 0.36）
#      cost_scaling_factor 25        （我之前 3.0，太软）
#      local costmap       global_frame=map, 4x4, 0.05
#      obstacle_range 2.5 / raytrace_range 3.0
#      global costmap      **不写 width/height**，让 static_layer 按地图自适应
#                          （我之前写死 8.0，正是"机器人在代价地图外"的根源）
#
#  ## 与他们的差异（必须改）
#
#    · 他们 remap /odom -> /odometry/filtered（EKF 输出），
#      我们没跑 EKF，用自己验证过的 /odom_clean
#    · 他们 robot_base_frame=base_link，我们统一用 base_footprint
#      （TF 里 base_footprint->base_link 是固定 0.06 高，两者等价）
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
NAV=$LXH/catkin_ws/src/carebot_nav_kit
P=$NAV/config/nav_shared
MAP=$NAV/maps/arena_shared.yaml
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 1. 停旧的建图/导航节点 ════════"
for p in slam_gmapping amcl move_base map_server; do pkill -x "$p" 2>/dev/null; done
sleep 4
echo "  已停"

echo ""
echo "════════ 2. 确认基础栈 ════════"
for t in /scan_filtered /imu_data /odom_clean; do
    printf '  %-16s ' "$t"
    hz=$(timeout 8 rostopic hz "$t" 2>&1 | grep -m1 average || echo '')
    echo "${hz:-★无数据}"
done

echo ""
echo "════════ 3. map_server（共享地图）════════"
setsid nohup rosrun map_server map_server "$MAP" __name:=map_server \
    > /tmp/m26_map.log 2>&1 < /dev/null &
sleep 8
timeout 10 rostopic list 2>/dev/null | grep -qx /map && echo "  OK" \
    || { echo "  FAIL"; cat /tmp/m26_map.log; exit 1; }

echo ""
echo "════════ 4. amcl（他们的参数）════════"
rosparam load "$P/amcl_params.yaml" /amcl
setsid nohup rosrun amcl amcl __name:=amcl scan:=/scan_filtered odom:=/odom_clean \
    > /tmp/m26_amcl.log 2>&1 < /dev/null &
sleep 10
echo "  amcl=$(pgrep -xc amcl 2>/dev/null || echo 0)"

echo ""
echo "════════ 5. move_base（他们的参数）════════"
rosparam delete /move_base 2>/dev/null
sleep 1
# 无包装键的文件 -> 带 ns
rosparam load "$P/costmap_common_params_global.yaml" /move_base/global_costmap
rosparam load "$P/costmap_common_params_local.yaml"  /move_base/local_costmap
# 有包装键的文件 -> 不带 ns，载到 /move_base
rosparam load "$P/global_costmap_params.yaml" /move_base
rosparam load "$P/local_costmap_params.yaml"  /move_base
rosparam load "$P/global_planner_params.yaml" /move_base
rosparam load "$P/dwa_local_planner_params.yaml" /move_base
rosparam set /move_base/base_global_planner global_planner/GlobalPlanner
rosparam set /move_base/base_local_planner  dwa_local_planner/DWAPlannerROS
# 我们的坐标系用 base_footprint（他们写的是 base_link）
rosparam set /move_base/global_costmap/robot_base_frame base_footprint
rosparam set /move_base/local_costmap/robot_base_frame  base_footprint

echo "  radius=$(rosparam get /move_base/global_costmap/robot_radius)  inflation=$(rosparam get /move_base/global_costmap/inflation_layer/inflation_radius)  cost_scaling=$(rosparam get /move_base/global_costmap/inflation_layer/cost_scaling_factor)"
echo "  local: frame=$(rosparam get /move_base/local_costmap/global_frame) size=$(rosparam get /move_base/local_costmap/width)"

setsid nohup rosrun move_base move_base __name:=move_base scan:=/scan_filtered \
    > /tmp/m26_mb.log 2>&1 < /dev/null &
sleep 25
printf '  /move_base/status: '
timeout 12 rostopic hz /move_base/status 2>&1 | grep -m1 average || echo '★无消息'
echo "  global costmap 实际范围:"
timeout 15 python -c "
from __future__ import print_function
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('c', anonymous=True)
try:
    m = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=12)
    i = m.info
    print('    %dx%d @%.3f origin(%.2f,%.2f) -> x %.2f~%.2f y %.2f~%.2f'
          % (i.width,i.height,i.resolution,i.origin.position.x,i.origin.position.y,
             i.origin.position.x, i.origin.position.x+i.width*i.resolution,
             i.origin.position.y, i.origin.position.y+i.height*i.resolution))
except Exception as e:
    print('    [FAIL] %r' % e)
" 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 6. TF ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo '--'
done
echo ""
echo "  完成 —— 下一步定位 + 发航点"
