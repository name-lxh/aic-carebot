#!/bin/bash
# ============================================================================
#  stage2_base_only.sh —— 起底盘驱动，验证 /odom（机器人应保持静止）
# ============================================================================
#  ⚠️ 仍然**不启动导航、不发运动指令**。
#     这一阶段只确认三件事：
#        1. 底盘串口通（/dev/stm32）
#        2. /odom 在发，且**静止时读数不漂**（这决定里程计能不能用）
#        3. 静止时 /cmd_vel 没人发 → 机器人不动
#
#  ⚠️ 启动底盘前**先确认机器人周围是空的**（人别站在边上）。
# ============================================================================
set -u

ROS_IP_OURS=192.168.43.233
export ROS_IP="$ROS_IP_OURS"
export ROS_MASTER_URI="http://${ROS_IP_OURS}:11399"

LOG=/tmp/stage2_base.log

echo "════════ 0. 环境 ════════"
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
echo "  ROS_IP=$ROS_IP"

echo ""
echo "════════ 1. roscore 还在吗 ════════"
if pgrep -x rosmaster >/dev/null; then
    echo "  ✔ roscore 在（PID $(pgrep -x rosmaster)）"
else
    echo "  · 起了新的 roscore"
    roscore > /tmp/stage2_roscore.log 2>&1 &
    sleep 8
fi

echo ""
echo "════════ 2. 雷达还在吗 ════════"
if timeout 8 rostopic list 2>/dev/null | grep -qx '/scan'; then
    echo "  ✔ /scan 在"
else
    echo "  · 重新起雷达"
    roslaunch lslidar_driver lslidar_serial.launch > /tmp/stage2_lidar.log 2>&1 &
    sleep 12
fi

echo ""
echo "════════ 3. 确认底盘串口 ════════"
ls -la /dev/stm32 2>&1 | sed 's/^/  /'
[ -e /dev/stm32 ] || { echo "  ✘ ★ 底盘串口不存在"; exit 1; }

echo ""
echo "════════ 4. ★ 起底盘驱动（只起，不发指令）════════"
echo "  launch 内容（确认端口/波特率）："
B=$(rospack find base_control 2>/dev/null)
grep -E 'stm32_port|baudrate|node name|type=' "$B/launch/stm32.launch" 2>/dev/null | sed 's/^/      /'

roslaunch base_control stm32.launch > "$LOG" 2>&1 &
sleep 10

echo ""
echo "  节点："
timeout 8 rosnode list 2>/dev/null | sed 's/^/      /'
echo "  底盘相关话题："
timeout 8 rostopic list 2>/dev/null | grep -iE 'odom|cmd_vel|battery|imu' | sed 's/^/      /'

echo ""
echo "════════ 5. ★ /odom 是否在发、静止时漂不漂 ════════"
if timeout 10 rostopic list 2>/dev/null | grep -qx '/odom'; then
    echo "  ✔ /odom 存在"
    echo "  频率：$(timeout 12 rostopic hz /odom 2>&1 | grep -m1 average || echo '（读不到）')"
    echo ""
    echo "  静止漂移测试（10 秒，机器人**不要碰**）："
    timeout 30 python /tmp/odom_drift.py 2>&1 | sed 's/^/      /'
else
    echo "  ✘ ★ 没有 /odom"
    echo "  底盘日志："
    tail -25 "$LOG" | sed 's/^/      /'
fi

echo ""
echo "════════ 6. ★ 确认没人发 /cmd_vel（静止保障）════════"
echo "  /cmd_vel 的发布者："
timeout 8 rostopic info /cmd_vel 2>&1 | sed -n '/Publishers/,/Subscribers/p' | sed 's/^/      /'

echo ""
echo "════════ 7. 结束 ════════"
echo "  底盘日志：$LOG"
echo "  ⚠️ 此时若 /odom 正常且无人发 /cmd_vel，机器人应当**一动不动**"
