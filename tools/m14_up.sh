#!/bin/bash
# ============================================================================
#  m14_up.sh —— 起基础传感器栈（不含 gmapping / amcl / move_base）
# ============================================================================
#  机器人重启过，ROS 全停。这一步按顺序把底座拉起来：
#      roscore -> 雷达 -> 底盘 -> IMU -> 静态TF -> odom_imu -> 激光滤波
#
#  ⚠️ 只用 `pkill -x`（精确进程名），**不要用 `pkill -f`**：
#     本脚本的命令行里含 "move_base"/"gmapping" 等词，
#     `pkill -f` 会把执行它的 shell 自己也匹配上 -> SSH 断连。
#     （这个坑本项目已经踩过 3 次）
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 0. 清理残留 ════════"
for p in slam_gmapping amcl move_base map_server laser_filter \
         lslidar_driver_node art_imu_01_node art_imu_node stm32_node; do
    pkill -x "$p" 2>/dev/null
done
pkill -f '[s]tatic_transform_publisher' 2>/dev/null
pkill -f '[o]dom_imu.py' 2>/dev/null
pkill -f '[o]dom_relay' 2>/dev/null
sleep 4
echo "  已清理"

echo ""
echo "════════ 1. roscore ════════"
if pgrep -x rosmaster >/dev/null 2>&1; then
    echo "  已在跑"
else
    setsid nohup roscore -p 11399 > /tmp/m14_roscore.log 2>&1 < /dev/null &
    sleep 8
fi
pgrep -x rosmaster >/dev/null && echo "  OK" || { echo "  FAIL"; tail -8 /tmp/m14_roscore.log; exit 1; }

echo ""
echo "════════ 2. 雷达 ════════"
setsid nohup roslaunch lslidar_driver lslidar_serial.launch \
    > /tmp/m14_lidar.log 2>&1 < /dev/null &
sleep 12
echo "  /scan: $(timeout 12 rostopic hz /scan 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 3. 底盘 ════════"
setsid nohup roslaunch base_control stm32.launch \
    > /tmp/m14_base.log 2>&1 < /dev/null &
sleep 10
echo "  /odom(raw): $(timeout 10 rostopic hz /odom 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 4. IMU（YESENSE @460800）════════"
setsid nohup roslaunch "$WS/src/carebot_nav_kit/launch/include/imu.launch" \
    > /tmp/m14_imu.log 2>&1 < /dev/null &
sleep 14
echo "  /imu_data: $(timeout 12 rostopic hz /imu_data 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 5. 静态 TF ════════"
setsid nohup rosrun tf static_transform_publisher 0 0 0.06 0 0 0 base_footprint base_link 50 \
    > /dev/null 2>&1 < /dev/null &
sleep 1
setsid nohup rosrun tf static_transform_publisher 0 0 0.21 0 0 0 base_link laser_link 50 \
    > /dev/null 2>&1 < /dev/null &
sleep 1
setsid nohup rosrun tf static_transform_publisher 0.125 0 0.11 0 0 0 base_link IMU_link 50 \
    > /dev/null 2>&1 < /dev/null &
sleep 2
echo "  base_footprint->laser_link: $(timeout 5 rosrun tf tf_echo base_footprint laser_link 2>/dev/null | grep -c Translation)"

echo ""
echo "════════ 6. 干净 odom 源 ════════"
setsid nohup python "$LXH/_tools/odom_imu.py" \
    > /tmp/m14_odom.log 2>&1 < /dev/null &
sleep 8
echo "  odom->base_footprint: $(timeout 6 rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -c Translation)"
tail -2 /tmp/m14_odom.log 2>/dev/null | sed 's/^/    /'

echo ""
echo "════════ 7. 激光滤波 ════════"
setsid nohup roslaunch "$WS/src/carebot_nav_kit/launch/include/laser_filter.launch" \
    > /tmp/m14_filter.log 2>&1 < /dev/null &
sleep 12
echo "  /scan_filtered: $(timeout 12 rostopic hz /scan_filtered 2>&1 | grep -m1 average || echo '★无数据')"
timeout 20 python -c "
from __future__ import print_function
import rospy
from sensor_msgs.msg import LaserScan
rospy.init_node('m14chk', anonymous=True)
for t in ('/scan','/scan_filtered'):
    s = rospy.wait_for_message(t, LaserScan, timeout=10)
    fin=[v for v in s.ranges if v==v and v!=float('inf') and v>0.02]
    print('    %-15s valid=%d min=%.3f' % (t, len(fin), min(fin) if fin else -1))
" 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 8. 完成 ════════"
timeout 10 rosnode list 2>/dev/null | grep -vE 'tf_echo' | sed 's/^/  /'
echo ""
echo "  机器人**未移动**（本次没发任何 /cmd_vel）"
