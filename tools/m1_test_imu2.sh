#!/bin/bash
# 测试 YESENSE IMU 启动（art_imu_01_node @460800）
# ⚠️ pkill 必须用 -x（精确进程名）：
#    用 `pkill -f art_imu` 会匹配到**执行这条命令的 shell 自己**
#    （因为命令串里含 "art_imu"），把自己杀掉 → SSH 连接断。
set -u
source /home/ub/Hajimi/LXH/env.sh

L=/home/ub/Hajimi/LXH/catkin_ws/src/carebot_nav_kit/launch/include/imu.launch

echo "════════ 0. 清理旧进程（用 pkill -x 避免自杀）════════"
pkill -x art_imu_node     2>/dev/null
pkill -x art_imu_01_node  2>/dev/null
sleep 3
echo "  art_imu_node:    $(pgrep -xc art_imu_node 2>/dev/null || echo 0)"
echo "  art_imu_01_node: $(pgrep -xc art_imu_01_node 2>/dev/null || echo 0)"

echo ""
echo "════════ 1. 包解析确认 ════════"
echo "  art_imu_01 -> $(rospack find art_imu_01 2>/dev/null)"
echo "  节点二进制:"
ls -la "$(rospack find art_imu_01)/../devel/lib/art_imu_01/" 2>/dev/null | sed 's/^/    /'
echo "  launch: $L  $([ -f "$L" ] && echo 存在 || echo 缺失)"

echo ""
echo "════════ 2. 起 imu.launch ════════"
setsid nohup roslaunch "$L" > /tmp/m1_imu4.log 2>&1 < /dev/null &
sleep 16

echo "  进程:"
pgrep -ax art_imu_01_node 2>/dev/null | sed 's/^/    /' || echo "    (无)"
pgrep -ax art_imu_node 2>/dev/null | sed 's/^/    /'

echo ""
echo "  日志:"
head -25 /tmp/m1_imu4.log 2>/dev/null | sed 's/^/    /'

echo ""
echo "════════ 3. /imu_data ════════"
if timeout 12 rostopic list 2>/dev/null | grep -qx '/imu_data'; then
    echo "  [OK] 话题存在"
    echo "  频率: $(timeout 12 rostopic hz /imu_data 2>&1 | grep -m1 average || echo '无数据')"
    echo ""
    echo "  一帧（看 orientation 是不是真的在变、角速度是否合理）:"
    timeout 10 rostopic echo -n1 /imu_data 2>/dev/null | head -22 | sed 's/^/    /'
else
    echo "  [FAIL] 没有 /imu_data"
fi

echo ""
echo "════════ 4. 所有话题 ════════"
timeout 10 rostopic list 2>/dev/null | sed 's/^/  /'
