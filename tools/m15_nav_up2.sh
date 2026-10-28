#!/bin/bash
# ============================================================================
#  m15_nav_up2.sh —— 用 arena_nav（入口保留可通行）起导航栈
# ============================================================================
#  ⚠️ 参数命名空间（踩过两次）：
#      costmap_common_params.yaml  **无**包装键 -> 带 ns
#      global/local_costmap_params **有**包装键 -> 不带 ns（载到 /move_base）
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
NAV=$LXH/catkin_ws/src/carebot_nav_kit
MAP=$NAV/maps/arena_nav.yaml
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 1. 停旧导航节点 ════════"
for p in slam_gmapping amcl move_base map_server; do pkill -x "$p" 2>/dev/null; done
sleep 4
echo "  已停"

echo ""
echo "════════ 2. map_server（arena_nav）════════"
setsid nohup rosrun map_server map_server "$MAP" __name:=map_server \
    > /tmp/m15_map.log 2>&1 < /dev/null &
sleep 8
timeout 10 rostopic list 2>/dev/null | grep -qx /map && echo "  OK /map" || { echo "  FAIL"; cat /tmp/m15_map.log; exit 1; }

echo ""
echo "════════ 3. amcl ════════"
rosparam load "$NAV/config/nav/amcl_params.yaml" /amcl
setsid nohup rosrun amcl amcl __name:=amcl scan:=/scan_filtered odom:=/odom_clean \
    > /tmp/m15_amcl.log 2>&1 < /dev/null &
sleep 10
echo "  amcl 进程: $(pgrep -xc amcl 2>/dev/null || echo 0)"

echo ""
echo "════════ 4. move_base ════════"
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
# ⚠️ 这里**不再**硬编码覆盖 robot_radius / max_vel_x
#    之前写死 0.25 / 0.12，导致配置文件里改的值全被覆盖掉，
#    调参永远不生效（踩过一次）。
#    现在完全以 config/nav/*.yaml 为准。
echo "  width=$(rosparam get /move_base/global_costmap/width)  origin_x=$(rosparam get /move_base/global_costmap/origin_x)  radius=$(rosparam get /move_base/global_costmap/robot_radius)  vmax=$(rosparam get /move_base/DWAPlannerROS/max_vel_x)"

setsid nohup rosrun move_base move_base __name:=move_base scan:=/scan_filtered \
    > /tmp/m15_mb.log 2>&1 < /dev/null &
sleep 25

echo ""
echo "════════ 5. 用真实数据流验证 ════════"
printf '  /move_base/status: '
timeout 12 rostopic hz /move_base/status 2>&1 | grep -m1 average || echo '★无消息'
timeout 15 python -c "
from __future__ import print_function
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('m15v', anonymous=True)
m = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=12)
i = m.info
print('  costmap %dx%d @%.3f origin(%.2f,%.2f) 覆盖 x %.2f~%.2f y %.2f~%.2f'
      % (i.width,i.height,i.resolution,i.origin.position.x,i.origin.position.y,
         i.origin.position.x, i.origin.position.x+i.width*i.resolution,
         i.origin.position.y, i.origin.position.y+i.height*i.resolution))
" 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 6. 报错 ════════"
grep -iE 'error|died|fatal' /tmp/m15_mb.log 2>/dev/null | tail -4 | sed 's/^/  /' || echo "  (无)"
echo ""
echo "  完成"
