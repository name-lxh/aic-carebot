#!/bin/bash
# 看 base_control（底盘驱动）怎么工作 —— 重点是"上电后电机会不会立刻动"
set +u
source /opt/ros/kinetic/setup.bash
[ -f /home/ub/catkin_ws/devel/setup.bash ] && source /home/ub/catkin_ws/devel/setup.bash
set -u

BC=$(rospack find base_control 2>/dev/null)
echo "════════ base_control 包结构 ════════"
echo "  $BC"
find "$BC" -maxdepth 2 -type f \( -name '*.py' -o -name '*.launch' -o -name '*.cpp' -o -name '*.yaml' \) 2>/dev/null | sed 's/^/      /'

echo ""
echo "════════ stm32.launch ════════"
cat -n "$BC/launch/stm32.launch" 2>/dev/null

echo ""
echo "════════ ★ 源码里的"使能/上电/急停"相关逻辑 ════════"
grep -rn -iE 'enable|brake|estop|e_stop|emergency|safe|watchdog|timeout' \
    "$BC" --include='*.py' --include='*.cpp' --include='*.h' 2>/dev/null \
    | head -30 | sed 's/^/      /'

echo ""
echo "════════ ★ 订阅/发布的话题（cmd_vel 从哪来、odom 往哪发）════════"
grep -rn -E 'Subscriber|Publisher|advertise|subscribe|cmd_vel|odom' \
    "$BC" --include='*.py' --include='*.cpp' 2>/dev/null \
    | head -30 | sed 's/^/      /'

echo ""
echo "════════ ★ 速度限幅（第一次测试要看这个）════════"
grep -rn -iE 'max_|min_|clamp|limit|scale|speed|velocity' \
    "$BC" --include='*.py' --include='*.cpp' --include='*.yaml' 2>/dev/null \
    | head -25 | sed 's/^/      /'
