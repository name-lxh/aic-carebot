#!/bin/bash
# kill_all_ros.sh —— 清掉车上所有 ROS 进程（含其他组的），给干净启动让路
#
# 其它组常驻的栈：ekf_se / simple_gmapping / move_base / driver_runtime /
# view_navigation / teleop / astra ... 两套栈同时发 map->odom 或抢相机/串口，
# 会让定位和识别互相污染。比赛前必须清干净。
set -u
echo "=== 清理前 ==="
echo "  roslaunch: $(pgrep -fc 'bin/[r]oslaunch' 2>/dev/null || echo 0)"
echo "  rosmaster: $(pgrep -fc 'bin/[r]osmaster' 2>/dev/null || echo 0)"

for p in $(pgrep -f 'bin/[r]oslaunch' 2>/dev/null); do kill "$p" 2>/dev/null; done
sleep 1
for p in $(pgrep -f 'bin/[r]osmaster' 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f 'bin/[r]oscore' 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 1

for pat in rosrun nodelet move_base amcl map_server slam_gmapping ekf_localization \
           stm32 lslidar_driver art_imu odom_imu qr_detector face_gate iot_bridge \
           tts_net laser_filter static_transform_publisher astra camera_nodelet \
           rviz teleop_twist driver_runtime ld_navigation sd_navigation; do
    for p in $(pgrep -f "[${pat:0:1}]${pat:1}" 2>/dev/null); do
        kill -9 "$p" 2>/dev/null
    done
done
sleep 2
echo "=== 清理后 ==="
echo "  roslaunch: $(pgrep -fc 'bin/[r]oslaunch' 2>/dev/null || echo 0)"
echo "  rosmaster: $(pgrep -fc 'bin/[r]osmaster' 2>/dev/null || echo 0)"
echo "  ROS 相关: $(ps aux | grep -cE '[r]os[a-z]*|[n]odelet' 2>/dev/null || echo 0)"
