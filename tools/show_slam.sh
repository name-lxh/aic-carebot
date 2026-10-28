#!/bin/bash
# 看组委会的 gmapping launch 怎么配的，以及 TF 链缺什么
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
source /home/ub/catkin_ws/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

N=$(rospack find carebot_navigation)/launch
echo "════════ gmapping_ls01d_lidar.launch ════════"
cat "$N/gmapping_ls01d_lidar.launch" 2>/dev/null | sed 's/^/  /'

echo ""
echo "════════ gmapping_slam.launch ════════"
cat "$N/gmapping_slam.launch" 2>/dev/null | sed 's/^/  /'

echo ""
echo "════════ ★ 当前 TF 树（缺什么）════════"
timeout 12 rosrun tf tf_monitor 2>&1 | tail -25 | sed 's/^/  /'

echo ""
echo "════════ ★ 现在有哪些 tf 相关话题/发布者 ════════"
timeout 8 rostopic list 2>/dev/null | grep -E '^/tf' | sed 's/^/  /'
echo "  --- /tf 的发布者 ---"
timeout 8 rostopic info /tf 2>&1 | sed -n '/Publishers/,/Subscribers/p' | sed 's/^/      /'
echo "  --- /tf_static 的发布者 ---"
timeout 8 rostopic info /tf_static 2>&1 | sed -n '/Publishers/,/Subscribers/p' | sed 's/^/      /'

echo ""
echo "════════ ★ odom 帧到底有没有被广播 ════════"
echo "  stm32.launch 里 is_pub_odom_tf 的值："
grep -n 'is_pub_odom_tf' $(rospack find base_control)/launch/stm32.launch | sed 's/^/      /'
echo "  （false = 不广播 odom->base_footprint，需要我们自己补静态 TF）"
