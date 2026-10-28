#!/bin/bash
# ============================================================================
#  m1_hw_probe.sh —— 建图第一阶段：起硬件，实测传感器质量（机器人静止）
# ============================================================================
#  ⚠️ 为什么先不直接跑 mapping.launch
#
#  gmapping 需要一条 **odom → base_footprint** 的 TF，而 odom 的质量
#  直接决定地图会不会扭曲。上次实地测试发现过：
#      · `/odom` 位置是 STM32 报的**绝对编码器值**（跑到 -126 ~ -213 m）
#      · 静止 10 s 航向漂 **44°**
#  如果这次还是这样，直接用 /odom 建图会得到一张烂图。
#
#  所以顺序是：
#      1. 只起 底盘 + 雷达 + IMU（**不启动 EKF、不启动 gmapping**）
#      2. 实测 /odom 的静止漂移、/imu 的航向稳定性、/scan 的规格
#      3. 根据实测结果再决定 odom 从哪来
#
#  ⚠️ 本阶段**不发任何速度指令**，机器人不会动。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
LOG=/tmp/m1_*.log

echo "════════ 0. 现场状态 ════════"
pgrep -x rosmaster >/dev/null 2>&1 && { echo "  [!] 已有 roscore，先停掉"; pkill -f roslaunch; pkill -x rosmaster; sleep 3; }
echo "  roscore: 无"
echo "  负载: $(uptime | sed 's/.*load average: //')"
echo "  串口:"
for d in /dev/stm32 /dev/carebot_laser_ls /dev/art_imu; do
    printf '    %-26s ' "$d"
    [ -e "$d" ] && echo "-> $(readlink -f $d)" || echo "★缺失"
done

echo ""
echo "════════ 1. 起 roscore ════════"
source "$LXH/env.sh"
setsid nohup roscore > /tmp/m1_roscore.log 2>&1 < /dev/null &
sleep 8
pgrep -x rosmaster >/dev/null && echo "  OK  roscore PID $(pgrep -x rosmaster)" || {
    echo "  [FAIL] roscore 没起来"; tail -12 /tmp/m1_roscore.log | sed 's/^/      /'; exit 1; }
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"

echo ""
echo "════════ 2. 雷达（LSLIDAR M10 @ /dev/carebot_laser_ls）════════"
setsid nohup roslaunch lslidar_driver lslidar_serial.launch > /tmp/m1_lidar.log 2>&1 < /dev/null &
sleep 12
echo "  /scan: $(timeout 10 rostopic hz /scan 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 3. 底盘（stm32 @ /dev/stm32）════════"
setsid nohup roslaunch base_control stm32.launch > /tmp/m1_base.log 2>&1 < /dev/null &
sleep 10
echo "  /odom: $(timeout 10 rostopic hz /odom 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 4. IMU（art_imu @ /dev/art_imu）════════"
setsid nohup roslaunch art_imu_01 imu_serial.launch > /tmp/m1_imu.log 2>&1 < /dev/null &
sleep 12
echo "  /imu_data: $(timeout 10 rostopic hz /imu_data 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 5. 节点清单 ════════"
timeout 10 rosnode list 2>/dev/null | sed 's/^/  /'

echo ""
echo "════════ 6. TF 现状（此时应该没有 odom→base_footprint）════════"
timeout 10 rostopic list 2>/dev/null | grep -E '^/tf' | sed 's/^/  /'
echo "  stm32 是否广播 odom TF:"
grep -o 'is_pub_odom_tf" *value="[a-z]*"' \
    "$(rospack find base_control)/launch/stm32.launch" 2>/dev/null | sed 's/^/    /'

echo ""
echo "════════ 7. ★ 传感器质量实测（机器人必须静止）════════"
echo "  7.1 /odom 静止漂移（12 秒）:"
timeout 40 python /tmp/m1_odom_check.py 2>&1 | sed 's/^/      /'

echo ""
echo "  7.2 /imu_data 航向稳定性（12 秒）:"
timeout 40 python /tmp/m1_imu_check.py 2>&1 | sed 's/^/      /'

echo ""
echo "  7.3 /scan 规格:"
timeout 40 python /tmp/m1_scan_check.py 2>&1 | sed 's/^/      /'

echo ""
echo "════════ 完成 ════════"
echo "  日志: /tmp/m1_lidar.log  /tmp/m1_base.log  /tmp/m1_imu.log"
echo "  ⚠️ 机器人**未移动**（本阶段没发过任何 /cmd_vel）"
