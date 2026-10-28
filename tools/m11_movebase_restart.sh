#!/bin/bash
# ============================================================================
#  m11_movebase_restart.sh —— 干净重启 move_base 并验证 action server 真的活着
# ============================================================================
#  ⚠️ 之前的教训：
#    `rosnode list` 和 `rostopic list` 里能看到 /move_base，
#    但进程其实**已经死了**，话题是残留注册。
#    于是 actionlib 的 wait_for_server() 一直失败 —— 看起来像"名字不对"，
#    其实是"服务端根本不存在"。
#
#  ✔ 所以本脚本用**真实数据流**判断存活：
#       /move_base/status 有没有消息（hz >= 1）
#     而不是只查进程/话题列表。
#
#  ⚠️ 参数命名空间必须分清（踩过：载错层级导致 width 不生效）：
#      costmap_common_params.yaml   **无**包装键 -> 带 ns 载入 global/local_costmap
#      global_costmap_params.yaml   **有** global_costmap: 包装 -> 不带 ns 载入 /move_base
#      local_costmap_params.yaml    **有** local_costmap:  包装 -> 不带 ns 载入 /move_base
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
NAV=$LXH/catkin_ws/src/carebot_nav_kit
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 1. 全停 move_base ════════"
pkill -x move_base 2>/dev/null
pkill -x move_base_ 2>/dev/null
sleep 4
echo "  残留: $(pgrep -xc move_base 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. 载入参数（注意命名空间）════════"
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
rosparam set /move_base/DWAPlannerROS/max_vel_x 0.20
rosparam set /move_base/DWAPlannerROS/max_vel_theta 0.6
rosparam set /move_base/global_costmap/robot_radius 0.25
rosparam set /move_base/local_costmap/robot_radius 0.25

echo "  关键参数:"
for p in global_costmap/width global_costmap/height global_costmap/origin_x \
         global_costmap/origin_y global_costmap/global_frame \
         global_costmap/robot_radius local_costmap/width local_costmap/robot_radius; do
    printf '    %-34s ' "$p"
    rosparam get "/move_base/$p" 2>/dev/null
done

echo ""
echo "════════ 3. 起 move_base ════════"
setsid nohup rosrun move_base move_base __name:=move_base scan:=/scan_filtered \
    > /tmp/m11_mb.log 2>&1 < /dev/null &
echo "  已启动，等 25 s 初始化"
sleep 25

echo ""
echo "════════ 4. ★ 用真实数据流判断存活 ════════"
printf '  进程存在: '
pgrep -xc move_base >/dev/null 2>&1 && echo YES || echo NO
printf '  /move_base/status 消息: '
timeout 12 rostopic hz /move_base/status 2>&1 | grep -m1 average || echo '★无消息（服务端不在）'

echo ""
echo "  代价地图覆盖:"
timeout 15 python -c "
from __future__ import print_function
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('cmv', anonymous=True)
try:
    m = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=12)
    i = m.info
    print('    %dx%d @%.3f origin(%.2f,%.2f)' % (i.width,i.height,i.resolution,i.origin.position.x,i.origin.position.y))
    print('    覆盖 x %.2f~%.2f  y %.2f~%.2f' % (i.origin.position.x, i.origin.position.x+i.width*i.resolution, i.origin.position.y, i.origin.position.y+i.height*i.resolution))
except Exception as e:
    print('    [FAIL] %r' % e)
" 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 5. 最近的报错 ════════"
grep -iE 'error|died|Failed|fatal' /tmp/m11_mb.log 2>/dev/null | tail -6 | sed 's/^/  /' || echo "  (无)"

echo ""
echo "════════ 完成 ════════"
