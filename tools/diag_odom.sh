#!/bin/bash
# 查 /odom 为什么是 -169/-180 且静止漂移大
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

echo "════════ 0. ★ 机器人现在到底动没动（连测 3 次间隔 3 秒）════════"
for i in 1 2 3; do
    timeout 8 rostopic echo -n1 /odom/pose/pose/position 2>/dev/null | tr '\n' ' ' | sed 's/^/  位置: /'
    echo ""
    timeout 8 rostopic echo -n1 /odom/twist/twist/linear 2>/dev/null | tr '\n' ' ' | sed 's/^/  线速度: /'
    echo ""
    sleep 3
done

echo ""
echo "════════ 1. 有没有别的节点在发 /cmd_vel ════════"
timeout 8 rostopic info /cmd_vel 2>&1 | sed 's/^/  /'

echo ""
echo "════════ 2. ★ 底盘驱动的里程计计算逻辑 ════════"
B=$(rospack find base_control)/script/stm32.py
grep -n -E 'pose_x|pose_y|self\.pose|def timerOdomCB' "$B" | head -20 | sed 's/^/  /'

echo ""
echo "--- timerOdomCB 全文 ---"
awk '/def timerOdomCB/,/^    def [a-z]/' "$B" | head -60 | cat -n | sed 's/^/  /'

echo ""
echo "━━━━━━━━ 3. 初始化里 pose 的初值 ━━━━━━━━"
grep -n -E 'self\.pose_x|self\.pose_y|self\.yaw|self\.th|self\.vx|self\.vy' "$B" | head -20 | sed 's/^/  /'

echo ""
echo "════════ 4. 串口读到的原始数据长什么样 ════════"
tail -40 /tmp/stage2_base.log | sed 's/^/  /'
