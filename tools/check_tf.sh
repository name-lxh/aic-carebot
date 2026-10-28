#!/bin/bash
# 确认 TF 链状态：map/odom/base_footprint/laser_link 通不通
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

echo "════════ 1. /tf 与 /tf_static 有没有实际数据流 ════════"
echo "  --- /tf 频率 ---"
timeout 8 rostopic hz /tf 2>&1 | head -3 | sed 's/^/      /'
echo "  --- /tf_static 频率 ---"
timeout 8 rostopic hz /tf_static 2>&1 | head -3 | sed 's/^/      /'

echo ""
echo "════════ 2. /tf 里到底在广播什么变换 ════════"
timeout 8 rostopic echo -n3 /tf 2>&1 | grep -E 'frame_id|child_frame_id' | head -20 | sed 's/^/  /'

echo ""
echo "════════ 3. ★ 逐对检查 TF 链 ════════"
for pair in "odom base_footprint" "base_footprint laser_link" \
            "base_footprint base_link" "map odom"; do
    set -- $pair
    echo "  --- $1 -> $2 ---"
    timeout 6 rosrun tf tf_echo "$1" "$2" 2>&1 | grep -E 'Translation|Rotation|Exception|Could not' | head -3 | sed 's/^/      /'
done

echo ""
echo "════════ 4. 现有节点 ════════"
timeout 8 rosnode list 2>/dev/null | sed 's/^/  /'

echo ""
echo "════════ 5. 雷达和 imu 还在发吗 ════════"
for t in /scan /odom /imu_data; do
    printf '  %-12s ' "$t"
    timeout 8 rostopic hz "$t" 2>&1 | grep -m1 average || echo '（没有数据）'
done

echo ""
echo "════════ 6. carebot_description（URDF 会广播静态 TF）════════"
D=$(rospack find carebot_description 2>/dev/null)
ls "$D/launch" 2>/dev/null | sed 's/^/      /'
echo "  --- carebot_description.launch ---"
head -30 "$D/launch/carebot_description.launch" 2>/dev/null | sed 's/^/      /'
