#!/bin/bash
# 找"IMU 里程计 / EKF 融合"的现成方案（组委会工作空间里）
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
source /home/ub/catkin_ws/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

echo "════════ 1. TF 树现状（谁是 odom 的来源）════════"
timeout 10 rosrun tf tf_monitor 2>&1 | head -25 | sed 's/^/  /'

echo ""
echo "════════ 2. 和 /odom 相关的发布者 ════════"
timeout 8 rostopic info /odom 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 3. ★ odom_ekf.launch / stm32_imu_ekf.launch 内容 ════════"
CB=$(rospack find carebot_bringup 2>/dev/null)/launch
CN=$(rospack find carebot_navigation 2>/dev/null)/launch
echo "  --- carebot_bringup/odom_ekf.launch ---"
cat "$CB/odom_ekf.launch" 2>/dev/null | sed 's/^/      /'
echo ""
echo "  --- carebot_navigation/stm32_imu_ekf.launch ---"
cat "$CN/stm32_imu_ekf.launch" 2>/dev/null | sed 's/^/      /'

echo ""
echo "════════ 4. ★ 组委会有没有现成的 IMU 里程计节点 ════════"
echo "  --- carebot_msgs 里有没有 odom 相关消息 ---"
CM=$(rospack find carebot_msgs 2>/dev/null)
ls "$CM/msg" 2>/dev/null | sed 's/^/      /'
echo "  --- carebot_bringup/odom_ekf.py 是什么 ---"
head -40 "$CB/odom_ekf.py" 2>/dev/null | sed 's/^/      /'

echo ""
echo "════════ 5. 全盘搜"imu"+"odom"的自研节点 ════════"
grep -rln --include='*.py' --include='*.cpp' \
    -iE 'imu.*odom|odom.*imu' /home/ub/omniWheelCareRobot/rosCode/src 2>/dev/null \
    | head -10 | sed 's/^/  /'

echo ""
echo "════════ 6. ekf_params.yaml（robot_localization 的配置）════════"
EP=$(rospack find robot_localization 2>/dev/null)/params/ekf_params.yaml
[ -f "$EP" ] && cat "$EP" | sed 's/^/      /' || echo "  （没有）"
