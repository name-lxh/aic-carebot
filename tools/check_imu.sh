#!/bin/bash
# 检查 IMU 可用性 —— 决定能不能用 IMU 航向替代不可信的轮式里程计
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

echo "════════ 1. IMU 设备 ════════"
ls -la /dev/art_imu /dev/fdilink_ahrs 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 2. art_imu_01 的 launch ════════"
AI=$(rospack find art_imu_01 2>/dev/null)
echo "  包目录: $AI"
ls "$AI/launch" 2>/dev/null | sed 's/^/      /'
echo "  --- imu_serial.launch ---"
cat "$AI/launch/imu_serial.launch" 2>/dev/null | sed 's/^/      /'

echo ""
echo "════════ 3. IMU 话题在不在 ════════"
timeout 8 rostopic list 2>/dev/null | grep -iE 'imu' | sed 's/^/  /' || echo "  · 没有 imu 话题"

echo ""
echo "════════ 4. ★ 起 IMU（如果没起）════════"
if ! timeout 8 rostopic list 2>/dev/null | grep -qi imu; then
    echo "  · 起 art_imu_01"
    roslaunch art_imu_01 imu_serial.launch > /tmp/imu.log 2>&1 &
    sleep 12
fi
echo "  现在的 imu 话题："
timeout 8 rostopic list 2>/dev/null | grep -iE 'imu' | sed 's/^/      /'

echo ""
echo "════════ 5. ★ IMU 静止漂移测试（30 秒）════════"
timeout 60 python /tmp/imu_drift.py 2>&1 | sed 's/^/  /'
