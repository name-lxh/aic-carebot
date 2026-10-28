#!/bin/bash
# 看雷达在车体上的安装位置，以及有没有遮挡
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u

D=$(rospack find carebot_description 2>/dev/null)
echo "════════ carebot_description = $D ════════"
find "$D" -name '*.urdf' -o -name '*.xacro' 2>/dev/null | sed 's/^/  /'

for f in $(find "$D" -name '*.urdf' 2>/dev/null); do
    echo ""
    echo "════════ $f ════════"
    echo "  行数: $(wc -l < "$f")"
    echo "  --- link 列表 ---"
    grep -oE '<link name="[^"]+"' "$f" | sed 's/<link name=//' | tr -d '"' | sed 's/^/      /'
    echo "  --- laser / scan 相关 ---"
    grep -n -i -B2 -A8 'laser' "$f" | head -40 | sed 's/^/      /'
done

echo ""
echo "════════ 雷达实车安装位置（从 launch / 静态 TF 反推）════════"
export ROS_IP=192.168.43.233
echo "  map -> laser_link:"
timeout 8 rosrun tf tf_echo map laser_link 2>&1 | grep -E 'Translation|Rotation' | sed 's/^/      /'
echo "  base_footprint -> laser_link:"
timeout 8 rosrun tf tf_echo base_footprint laser_link 2>&1 | grep -E 'Translation|Rotation' | sed 's/^/      /'
echo "  base_footprint -> IMU_link:"
timeout 8 rosrun tf tf_echo base_footprint IMU_link 2>&1 | grep -E 'Translation|Rotation' | sed 's/^/      /'
