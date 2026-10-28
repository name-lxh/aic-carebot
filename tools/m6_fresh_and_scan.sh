#!/bin/bash
# ============================================================================
#  m6_fresh_and_scan.sh —— 场地被撞、机器人被搬动后：重开新图 + 5 分钟扫描
# ============================================================================
#  ⚠️ 为什么必须重开地图
#
#  用户报告：小车撞墙并把墙**推动了一段距离**，之后手动把车拉回、
#  放到了空旷位置（四周分别是 B/C/D/A 四个房间）。
#
#  这带来两个必须处理的问题：
#
#    1. **场地变了**：墙被推动 → 旧地图里的墙体位置已经不对了。
#    2. **机器人被搬动**：`odom_imu.py` 的平移是靠 `/cmd_vel` 累积的，
#       被人抱起来移动时**不会累积**，于是 odom 会严重偏离真实位置；
#       gmapping 把这个错误 odom 当先验，地图会被拉歪。
#
#  → 所以：**重启 odom_imu（原点归零）+ 重启 gmapping（清空旧图）**，
#    以机器人当前所在位置建立全新的地图坐标系。
#
#  ⚠️ 旧地图文件 real_arena.* 保留不删（万一有用），新图另存为 real_arena2.*
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 0. 现状 ════════"
echo "  rosmaster: $(pgrep -xc rosmaster 2>/dev/null || echo 0)"
echo "  gmapping : $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"

echo ""
echo "════════ 1. 停掉 gmapping 和 odom_imu（保留雷达/底盘/IMU/滤波）════════"
pkill -x slam_gmapping 2>/dev/null
pkill -f '[o]dom_imu.py' 2>/dev/null
sleep 4
echo "  gmapping=$(pgrep -xc slam_gmapping 2>/dev/null || echo 0)  odom_imu=$(pgrep -fc '[o]dom_imu.py' 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. 确认基础传感器仍在 ════════"
for t in /scan /scan_filtered /imu_data; do
    printf '  %-16s ' "$t"
    hz=$(timeout 8 rostopic hz "$t" 2>&1 | grep -m1 average || echo '')
    echo "${hz:-★无数据}"
done

echo ""
echo "════════ 3. 重起 odom_imu（原点归零到当前位置）════════"
setsid nohup python "$LXH/_tools/odom_imu.py" \
    > /tmp/m6_odom.log 2>&1 < /dev/null &
sleep 8
tail -3 /tmp/m6_odom.log 2>/dev/null | sed 's/^/  /'
echo "  odom→base_footprint: $(timeout 6 rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -c Translation)"

echo ""
echo "════════ 4. 起全新的 gmapping ════════"
GP=$WS/src/carebot_nav_kit/config/slam/gmapping_params.yaml
rosparam load "$GP" /slam_gmapping 2>/dev/null
echo "  参数: xmin=$(rosparam get /slam_gmapping/xmin 2>/dev/null) particles=$(rosparam get /slam_gmapping/particles 2>/dev/null) delta=$(rosparam get /slam_gmapping/delta 2>/dev/null)"
setsid nohup rosrun gmapping slam_gmapping __name:=slam_gmapping scan:=/scan_filtered \
    > /tmp/m6_gmapping.log 2>&1 < /dev/null &
sleep 16
echo "  gmapping 进程: $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"

echo ""
echo "════════ 5. 新地图应该几乎是空的 ════════"
timeout 45 python "$LXH/_tools/map_info.py" 2>&1 | head -6 | sed 's/^/  /'

echo ""
echo "════════ 6. 完整 TF 链 ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo "--"
done

echo ""
echo "  准备就绪 —— 下一步跑 5 分钟扫描"
