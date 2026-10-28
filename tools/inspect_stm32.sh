#!/bin/bash
# 看 stm32.py 的速度控制逻辑，判断"上电后会不会自己动"
set +u
source /opt/ros/kinetic/setup.bash
[ -f /home/ub/catkin_ws/devel/setup.bash ] && source /home/ub/catkin_ws/devel/setup.bash
set -u

B=$(rospack find base_control)/script/stm32.py
echo "文件: $B   行数: $(wc -l < "$B")"

echo ""
echo "════════ 1. 速度相关初值 ════════"
grep -n -E 'self\.(current_x|current_rot_z|speed|max_step_vx|max_step_vz|last_cmd_vel_time)\s*=' "$B" | sed 's/^/  /'

echo ""
echo "════════ 2. cmdCB（收到 /cmd_vel 时做什么）════════"
grep -n 'def cmdCB' -A 25 "$B" | sed 's/^/  /'

echo ""
echo "════════ 3. 速度斜坡/限幅的核心段（300-345 行）════════"
sed -n '300,345p' "$B" | cat -n | sed 's/^/  /'

echo ""
echo "════════ 4. 有没有"超时归零"（cmd_vel 断了会停吗）════════"
grep -n -E 'last_cmd_vel_time|timeout|rospy\.Time\.now\(\) -' "$B" | sed 's/^/  /'

echo ""
echo "════════ 5. 主循环发速度的地方 ════════"
grep -n -E 'write|send|self\.speed|Vx|Vyaw' "$B" | head -25 | sed 's/^/  /'
