#!/bin/bash
# 测试修好的 IMU 启动（art_imu @460800）
set -u
source /home/ub/Hajimi/LXH/env.sh

echo "════════ 0. 先停掉旧的 IMU 尝试 ════════"
pkill -f art_imu_node 2>/dev/null
pkill -f 'roslaunch art_imu_01' 2>/dev/null
sleep 3
echo "  残留 art_imu_node: $(pgrep -fc art_imu_node 2>/dev/null || echo 0)"

echo ""
echo "════════ 1. 起我们的 imu.launch ════════"
# ⚠️ 用**绝对路径**：roslaunch pkg include/imu.launch 这种带子目录的
#    相对写法在 Kinetic 上解析不到，会报 "is neither a launch file..."。
L=/home/ub/Hajimi/LXH/catkin_ws/src/carebot_nav_kit/launch/include/imu.launch
echo "  launch: $L  $([ -f "$L" ] && echo '(存在)' || echo '(★不存在)')"
setsid nohup roslaunch "$L" > /tmp/m1_imu3.log 2>&1 < /dev/null &
sleep 15

echo "  进程:"
pgrep -af art_imu_node 2>/dev/null | head -3 | sed 's/^/    /' || echo "    (无)"

echo ""
echo "  日志关键行:"
grep -iE 'serial|baudrate|ERROR|Unable|initialized' /tmp/m1_imu3.log 2>/dev/null \
    | head -8 | sed 's/^/    /'

echo ""
echo "════════ 2. /imu_data ════════"
if timeout 12 rostopic list 2>/dev/null | grep -qx '/imu_data'; then
    echo "  话题存在"
    echo "  频率: $(timeout 12 rostopic hz /imu_data 2>&1 | grep -m1 average || echo '无数据')"
    echo ""
    echo "  一帧:"
    timeout 10 rostopic echo -n1 /imu_data 2>/dev/null | head -20 | sed 's/^/    /'
else
    echo "  [FAIL] 没有 /imu_data"
    tail -20 /tmp/m1_imu3.log 2>/dev/null | sed 's/^/    /'
fi

echo ""
echo "════════ 3. 节点清单 ════════"
timeout 10 rosnode list 2>/dev/null | sed 's/^/  /'
