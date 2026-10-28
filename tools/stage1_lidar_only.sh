#!/bin/bash
# ============================================================================
#  stage1_lidar_only.sh —— 只起雷达，验证硬件（机器人绝对静止）
# ============================================================================
#  ⚠️ 分阶段上电是实地测试的铁律：第一次只验证传感器，不启动底盘。
#
#  ⚠️⚠️ 本机必须显式设 ROS_IP —— 踩过的坑：
#
#     主机名是 `ub`，而 `/etc/hosts` 里有一条**陈旧的静态映射**
#         192.168.8.213   ub
#     那是**旧网络**（192.168.8.x）的地址，和当前网络
#     （wlan 192.168.43.233）对不上。
#
#     于是不设 ROS_IP 时，roscore 会去绑 192.168.8.213（本机没有这个地址），
#     **卡死且没有任何输出**（只有一句磁盘警告），看起来像"roscore 坏了"。
#
#     显式 `ROS_IP=192.168.43.233` 后立刻正常：
#         started roslaunch server http://192.168.43.233:44547/
#
#  ⚠️ 顺带清理 ~/.ros/log（已涨到 1.3GB，roscore 会警告且变慢）
# ============================================================================
set -u

ROS_IP_OURS=192.168.43.233
export ROS_IP="$ROS_IP_OURS"
export ROS_MASTER_URI="http://${ROS_IP_OURS}:11399"

LOG=/tmp/stage1_lidar.log
echo "════════ 0. 环境 ════════"
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
echo "  ROS_DISTRO=${ROS_DISTRO:-?}"
echo "  ROS_IP=$ROS_IP   （显式设，绕过 /etc/hosts 的陈旧映射）"
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"

echo ""
echo "════════ 1. 清理 ROS 日志（已 1.3GB）════════"
BEFORE=$(du -sh /home/ub/.ros/log 2>/dev/null | cut -f1)
rm -rf /home/ub/.ros/log/* 2>/dev/null
AFTER=$(du -sh /home/ub/.ros/log 2>/dev/null | cut -f1)
echo "  $BEFORE -> $AFTER"

echo ""
echo "════════ 2. 清理残留 ros 进程 ════════"
for p in roslaunch rosmaster roscore lslidar; do
    pkill -f "$p" 2>/dev/null
done
sleep 3
echo "  残留 roslaunch: $(pgrep -fc roslaunch 2>/dev/null || echo 0)"

echo ""
echo "════════ 3. 确认雷达串口 ════════"
ls -la /dev/carebot_laser_ls 2>&1 | sed 's/^/  /'
[ -e /dev/carebot_laser_ls ] || { echo "  ✘ ★ 雷达串口不存在"; exit 1; }

echo ""
echo "════════ 4. 起 roscore ════════"
roscore > /tmp/stage1_roscore.log 2>&1 &
sleep 8
if pgrep -x rosmaster >/dev/null; then
    echo "  ✔ roscore 在跑（PID $(pgrep -x rosmaster)）"
else
    echo "  ✘ ★ roscore 没起来："
    tail -20 /tmp/stage1_roscore.log | sed 's/^/      /'
    exit 1
fi

echo ""
echo "════════ 5. 起雷达驱动 ════════"
L=$(rospack find lslidar_driver 2>/dev/null)
echo "  可用 launch:"
ls "$L/launch" 2>/dev/null | sed 's/^/      /'

roslaunch lslidar_driver lslidar_serial.launch > "$LOG" 2>&1 &
sleep 15

echo ""
echo "  节点："
timeout 8 rosnode list 2>/dev/null | sed 's/^/      /'
echo "  话题："
timeout 8 rostopic list 2>/dev/null | grep -iE 'scan|lslidar' | sed 's/^/      /'

echo ""
echo "════════ 6. ★ /scan 规格核对 ════════"
if timeout 10 rostopic list 2>/dev/null | grep -qx '/scan'; then
    echo "  ✔ /scan 存在"
    echo "  频率：$(timeout 12 rostopic hz /scan 2>&1 | grep -m1 average || echo '（读不到）')"
    echo ""
    echo "  一帧规格："
    timeout 20 python /tmp/scan_spec.py 2>&1 | sed 's/^/      /'
else
    echo "  ✘ ★ 没有 /scan 话题"
    echo "  雷达日志尾部："
    tail -25 "$LOG" | sed 's/^/      /'
fi

echo ""
echo "════════ 7. 结束 ════════"
echo "  雷达日志：$LOG"
echo "  底盘**未启动**，机器人应完全静止"
