#!/bin/bash
# 重启 gmapping（用正确的场地参数），然后确认状态
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233
export ROS_MASTER_URI=http://192.168.43.233:11399

echo "════════ 1. 停掉旧的 gmapping ════════"
pkill -f slam_gmapping 2>/dev/null
sleep 4
echo "  残留: $(pgrep -fc slam_gmapping 2>/dev/null || echo 0)"
echo "  roscore: $(pgrep -x rosmaster >/dev/null && echo 在跑 || echo 没了)"

if ! pgrep -x rosmaster >/dev/null; then
    echo "  ★ roscore 没了（pkill 误杀？），重起整条链路"
    exit 2
fi

echo ""
echo "════════ 2. 用正确参数起 gmapping ════════"
nohup roslaunch /tmp/slam_arena.launch > /tmp/gm2.log 2>&1 &
sleep 14

echo "  进程: $(pgrep -fc slam_gmapping 2>/dev/null || echo 0)"
echo ""
echo "  --- 参数确认（应该是 ±4，不是 ±100）---"
grep -oE '\-maxUrange [0-9.]+|\-xmin -?[0-9.]+|\-particles [0-9]+|\-linearUpdate [0-9.]+' \
    /tmp/gm2.log | head -8 | sed 's/^/      /'

echo ""
echo "  --- 地图信息 ---"
timeout 40 python /tmp/map_info.py 2>&1 | head -12 | sed 's/^/      /'

echo ""
echo "════════ 3. 三个输入源 ════════"
for t in /scan /odom /imu_data; do
    printf '  %-12s ' "$t"
    timeout 8 rostopic hz "$t" 2>&1 | grep -m1 average || echo '（无数据）'
done

echo ""
echo "════════ 4. TF 链 ════════"
for pair in "odom base_footprint" "map odom"; do
    set -- $pair
    printf '  %-26s ' "$1 -> $2"
    timeout 6 rosrun tf tf_echo "$1" "$2" 2>&1 | grep -qE 'Translation|Rotation' \
        && echo "✔" || echo "✘"
done

echo ""
echo "════════ 5. 状态总结 ════════"
echo "  gmapping: $(pgrep -f slam_gmapping >/dev/null && echo '在跑' || echo '★没跑')"
echo "  日志: /tmp/gm2.log"
