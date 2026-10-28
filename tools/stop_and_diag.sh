#!/bin/bash
# 停掉探索 + 查那个持续的"前方 0.13m"
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

echo "════════ 1. 停掉探索 ════════"
pkill -f explore_map 2>/dev/null
sleep 3
echo "  残留 explore_map: $(pgrep -fc explore_map 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. 地图现状 ════════"
timeout 40 python /tmp/map_info.py 2>&1 | head -8 | sed 's/^/  /'

echo ""
echo "════════ 3. ★ 查那个持续的"前方 0.13m"到底是什么 ════════"
timeout 60 python /tmp/scan_detail.py 2>&1 | head -40 | sed 's/^/  /'
