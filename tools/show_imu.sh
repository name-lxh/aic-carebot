#!/bin/bash
# 看 /imu_data 的字段内容，确认有没有可用的绝对方位
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

echo "════════ /imu_data 的单帧内容 ════════"
timeout 12 rostopic echo -n1 /imu_data 2>&1 | head -60 | sed 's/^/  /'

echo ""
echo "════════ 其他 imu 话题看一下 ════════"
for t in /imu/nav_data /imu/inertial_data /imu/sensor_data; do
    echo "  --- $t ---"
    timeout 8 rostopic echo -n1 "$t" 2>&1 | head -12 | sed 's/^/      /'
done
