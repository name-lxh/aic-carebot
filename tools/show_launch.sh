#!/bin/bash
# 看组委会的 bringup / navigation launch 到底怎么配的（只读）
set +u
source /opt/ros/kinetic/setup.bash
[ -f /home/ub/catkin_ws/devel/setup.bash ] && source /home/ub/catkin_ws/devel/setup.bash
set -u

B=$(rospack find carebot_bringup)/launch/carebot_bringup.launch
echo "════════ carebot_bringup.launch ════════"
cat -n "$B" 2>/dev/null | head -80

echo ""
echo "════════ car_nav.launch（导航总入口？）════════"
N=$(rospack find carebot_navigation)/launch/car_nav.launch
cat -n "$N" 2>/dev/null | head -60

echo ""
echo "════════ run.sh（别人留的启动脚本）════════"
cat -n /home/ub/run.sh 2>/dev/null
