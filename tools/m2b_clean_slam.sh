#!/bin/bash
# ============================================================================
#  m2b_clean_slam.sh —— 干净地只起 gmapping（除掉 TF 重复 / EKF 冲突）
# ============================================================================
#  m2 用 `mapping.launch` 起 gmapping，结果冒出三个问题：
#
#    1. **TF 重复发布**：mapping.launch 内含 bringup.launch → static_tf.launch，
#       而 m2 自己又起了 3 个 static_transform_publisher。
#       同一对 frame 两个发布者 → rviz 抖动、地图扭曲。
#       （我们自己的 static_tf.launch 注释里就警告过这一点）
#
#    2. **/odom 两个来源**：mapping.launch 起了 EKF(ekf_se) + odom_relay
#       （/odometry/filtered → /odom），而 stm32_node 也在发 /odom。
#
#    3. **odom_monitor.py 崩溃**（已单独修好 argparse bug）。
#
#  所以本脚本改成**只起需要的**：
#       · 激光滤波  → /scan_filtered
#       · slam_gmapping
#  其余（/scan、/imu_data、静态TF、odom→base_footprint）由 m1/m2 提供，不重复起。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 0. 现状 ════════"
echo "  roscore: $(pgrep -xc rosmaster 2>/dev/null || echo 0)"
echo "  gmapping: $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"

echo ""
echo "════════ 1. 停掉 mapping.launch 起的那一堆 ════════"
# ⚠️ 用 pkill -x 精确匹配进程名，不用 -f（-f 会匹配到本脚本自己）
pkill -x slam_gmapping 2>/dev/null
pkill -x ekf_se 2>/dev/null
pkill -f 'odom_relay' 2>/dev/null
pkill -f 'odom_monitor' 2>/dev/null
pkill -x art_imu_node 2>/dev/null          # 旧的错版 IMU（会抢串口）
pkill -f 'roslaunch.*mapping.launch' 2>/dev/null
sleep 4
echo "  清理后 gmapping=$(pgrep -xc slam_gmapping 2>/dev/null || echo 0)  ekf=$(pgrep -xc ekf_se 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. 确认 m1/m2 的关键件还活着 ════════"
for t in /scan /imu_data; do
    printf '  %-12s ' "$t"
    timeout 8 rostopic list 2>/dev/null | grep -qx "$t" && echo "OK" || echo "FAIL"
done
printf '  %-12s ' "odom_imu"
pgrep -fc 'odom_imu.py' >/dev/null 2>&1 && echo "PID $(pgrep -f 'odom_imu.py' | head -1)" || echo "FAIL"
echo "  TF odom->base_footprint: $(timeout 5 rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -c Translation)"
echo "  TF base_footprint->laser_link: $(timeout 5 rosrun tf tf_echo base_footprint laser_link 2>/dev/null | grep -c Translation)"

echo ""
echo "════════ 3. 只起激光滤波 ════════"
LF=$WS/src/carebot_nav_kit/launch/include/laser_filter.launch
echo "  $LF"
setsid nohup roslaunch "$LF" > /tmp/m2b_filter.log 2>&1 < /dev/null &
sleep 10
echo "  /scan_filtered: $(timeout 10 rostopic hz /scan_filtered 2>&1 | grep -m1 average || echo '★无数据')"
grep -iE 'ERROR|died' /tmp/m2b_filter.log 2>/dev/null | head -3 | sed 's/^/    /'

echo ""
echo "════════ 4. 只起 gmapping ════════"
# 参数与 mapping.launch 里一致，但**不**连带 bringup/EKF/static_tf
GP=$WS/src/carebot_nav_kit/config/slam/gmapping_params.yaml
setsid nohup rosrun gmapping slam_gmapping \
    _odom_frame:=odom _base_frame:=base_footprint _map_frame:=map \
    __name:=slam_gmapping \
    scan:=/scan_filtered \
    > /tmp/m2b_gmapping.log 2>&1 < /dev/null &
sleep 6

# rosrun 的 _param:= 覆盖不稳，改用 rosparam load + respawn 方式：
# 直接把 yaml 的参数塞到节点命名空间再起
pkill -x slam_gmapping 2>/dev/null
sleep 2
rosparam load "$GP" /slam_gmapping 2>/dev/null
echo "  已把 $GP 载入 /slam_gmapping 命名空间"
rosparam get /slam_gmapping/xmin 2>/dev/null | sed 's/^/    xmin = /'
rosparam get /slam_gmapping/particles 2>/dev/null | sed 's/^/    particles = /'

setsid nohup rosrun gmapping slam_gmapping __name:=slam_gmapping scan:=/scan_filtered \
    > /tmp/m2b_gmapping.log 2>&1 < /dev/null &
sleep 14

echo ""
echo "  gmapping 进程: $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"
echo "  日志参数:"
grep -oE '\-xmin -?[0-9.]+|\-particles [0-9]+|\-delta [0-9.]+|\-maxUrange [0-9.]+' \
    /tmp/m2b_gmapping.log 2>/dev/null | head -6 | sed 's/^/    /'
echo "  错误:"
grep -iE 'ERROR|died|Failed' /tmp/m2b_gmapping.log 2>/dev/null | head -5 | sed 's/^/    /' || echo "    (无)"

echo ""
echo "════════ 5. /map 状态 ════════"
sleep 4
if timeout 12 rostopic list 2>/dev/null | grep -qx '/map'; then
    echo "  OK /map 存在"
    timeout 40 python /home/ub/Hajimi/LXH/_tools/map_info.py 2>&1 | head -8 | sed 's/^/    /'
else
    echo "  FAIL 没有 /map"
    tail -20 /tmp/m2b_gmapping.log 2>/dev/null | sed 's/^/    /'
fi

echo ""
echo "════════ 6. TF 完整链路 ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link" "base_footprint IMU_link"; do
    set -- $pair
    printf '  %-30s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo "--"
done

echo ""
echo "════════ 7. 节点（不应再有重复 TF / ekf）════════"
timeout 10 rosnode list 2>/dev/null | sed 's/^/  /'

echo ""
echo "════════ 完成（机器人在此阶段仍未移动）════════"
