#!/bin/bash
# ============================================================================
#  restart_slam2.sh —— 用修好的 odom（v2，命令积分）重起整条建图链路
# ============================================================================
#  ⚠️ v1 的 odom_imu 把 STM32 的绝对编码器值当平移喂给 gmapping，
#     导致地图膨胀到 18.2 x 18.2 m（原点跑到 -14.55）。
#     v2 改成对 /cmd_vel 积分 + 死区，**完全不看 /odom**。
#
#  重启顺序（每一步都验证，避免"看起来起了其实没通"）：
#     roscore → 雷达 → IMU → 底盘 → 静态 TF → odom_imu(v2) → gmapping
# ============================================================================
set -u

ROS_IP_OURS=192.168.43.233
export ROS_IP="$ROS_IP_OURS"
export ROS_MASTER_URI="http://${ROS_IP_OURS}:11399"

echo "════════ 0. 环境 ════════"
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
echo "  ROS_IP=$ROS_IP"

echo ""
echo "════════ 1. 全清 ════════"
for p in roslaunch rosmaster roscore lslidar stm32 art_imu odom_imu \
         slam_gmapping static_transform explore_map test_drive; do
    pkill -f "$p" 2>/dev/null
done
sleep 4
rm -rf /home/ub/.ros/log/* 2>/dev/null
echo "  残留 roslaunch: $(pgrep -fc roslaunch 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. roscore ════════"
roscore > /tmp/s2_roscore.log 2>&1 &
sleep 8
pgrep -x rosmaster >/dev/null && echo "  ✔" || { echo "  ✘"; exit 1; }

echo ""
echo "════════ 3. 雷达 ════════"
roslaunch lslidar_driver lslidar_serial.launch > /tmp/s2_lidar.log 2>&1 &
sleep 10
echo "  /scan $(timeout 8 rostopic hz /scan 2>&1 | grep -m1 average || echo '✘')"

echo ""
echo "════════ 4. IMU ════════"
roslaunch art_imu_01 imu_serial.launch > /tmp/s2_imu.log 2>&1 &
sleep 10
echo "  /imu_data $(timeout 8 rostopic hz /imu_data 2>&1 | grep -m1 average || echo '✘')"

echo ""
echo "════════ 5. 底盘 ════════"
roslaunch base_control stm32.launch > /tmp/s2_base.log 2>&1 &
sleep 8
echo "  /odom $(timeout 8 rostopic hz /odom 2>&1 | grep -m1 average || echo '✘')"

echo ""
echo "════════ 6. 静态 TF ════════"
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_link 100 > /dev/null 2>&1 &
sleep 1
rosrun tf static_transform_publisher 0 0 0.21 0 0 0 base_link laser_link 100 > /dev/null 2>&1 &
sleep 1
rosrun tf static_transform_publisher 0.125 0 0.11 0 0 0 base_link IMU_link 100 > /dev/null 2>&1 &
sleep 2
echo "  已起 3 个"

echo ""
echo "════════ 7. ★ odom_imu v2（命令积分）════════"
nohup python /tmp/odom_imu.py > /tmp/s2_odomimu.log 2>&1 &
sleep 8
tail -3 /tmp/s2_odomimu.log | sed 's/^/      /'

echo ""
echo "════════ 8. 检查 TF 链 ════════"
for pair in "base_footprint laser_link" "odom base_footprint"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 6 rosrun tf tf_echo "$1" "$2" 2>&1 | grep -q Translation \
        && echo "✔" || echo "✘"
done

echo ""
echo "════════ 9. gmapping（参数已按 3m 场地调好）════════"
nohup roslaunch /tmp/slam_arena.launch > /tmp/s2_gmapping.log 2>&1 &
sleep 14
echo "  进程: $(pgrep -fc slam_gmapping 2>/dev/null || echo 0)"
grep -oE '\-xmin -?[0-9.]+|\-particles [0-9]+' /tmp/s2_gmapping.log \
    | head -3 | sed 's/^/      /'

echo ""
echo "════════ 10. 地图（应仍是 256x256，因为 odom 不再乱跑）════════"
timeout 40 python /tmp/map_info.py 2>&1 | head -7 | sed 's/^/  /'

echo ""
echo "════════ 11. 完成 ════════"
echo "  gmapping: $(pgrep -f slam_gmapping >/dev/null && echo 在跑 || echo ★没跑)"
echo "  ⚠️ 机器人应完全静止（没人发 /cmd_vel）"
