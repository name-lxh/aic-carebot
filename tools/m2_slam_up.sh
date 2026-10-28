#!/bin/bash
# ============================================================================
#  m2_slam_up.sh —— 建图第二阶段：搭起 gmapping 完整链路
# ============================================================================
#  前置（m1 已完成并仍在跑）：
#      roscore、lslidar（/scan 10Hz）、stm32（/odom，不可信但底盘要它）、
#      art_imu_01_node（/imu_data 200Hz，航向 0.013°）
#
#  本阶段补上：
#      1. 静态 TF：base_footprint → base_link → {laser_link, IMU_link}
#      2. 干净的 odom 源：odom_imu.py
#           · 航向 = IMU（实测 12 s 漂 0.013°，可信）
#           · 平移 = /cmd_vel 命令积分（开环，会漂，但不跳变）
#           · 死区：|v|<0.01 当不动 → 原地旋转时平移严格为 0
#         为什么不用 stm32 的 /odom：
#           实测位置是**绝对编码器值**（x=314.010, y=-62.450，而场地才 3m），
#           静止 12 s 航向漂 **45°**，喂给 gmapping 会把地图拧成麻花。
#      3. gmapping（参数按 3m 小场地调过）
#
#  ⚠️ 本阶段仍**不发速度**，机器人不动。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
DISTRO=kinetic
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399

source "$LXH/env.sh"

echo "════════ 0. 前置检查 ════════"
ok=1
for t in /scan /imu_data; do
    if timeout 8 rostopic list 2>/dev/null | grep -qx "$t"; then
        printf '  OK   %-14s %s\n' "$t" "$(timeout 10 rostopic hz $t 2>&1 | grep -m1 average || echo '?')"
    else
        printf '  FAIL %-14s 不存在\n' "$t"; ok=0
    fi
done
[ "$ok" -eq 1 ] || { echo "  ★ 前置不全，请先跑 m1"; exit 1; }

echo ""
echo "════════ 1. 停掉可能冲突的东西 ════════"
# 只精确杀，不用 -f（避免匹配到自己这条命令）
pkill -x art_imu_node 2>/dev/null        # 旧的错版 IMU（若还在）
pkill -x slam_gmapping 2>/dev/null
pkill -f 'static_transform_publisher' 2>/dev/null
pkill -f 'odom_imu.py' 2>/dev/null
sleep 2
echo "  gmapping: $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. 静态 TF ════════"
# 数值与 carebot_nav_kit/config/robot/base.yaml 一致
setsid nohup rosrun tf static_transform_publisher 0 0 0.06 0 0 0 base_footprint base_link 50 \
    > /tmp/m2_tf1.log 2>&1 < /dev/null &
sleep 1
setsid nohup rosrun tf static_transform_publisher 0 0 0.21 0 0 0 base_link laser_link 50 \
    > /tmp/m2_tf2.log 2>&1 < /dev/null &
sleep 1
setsid nohup rosrun tf static_transform_publisher 0.125 0 0.11 0 0 0 base_link IMU_link 50 \
    > /tmp/m2_tf3.log 2>&1 < /dev/null &
sleep 2
echo "  起了 3 个静态 TF"

echo ""
echo "════════ 3. 干净 odom 源（odom_imu.py）════════"
setsid nohup python /home/ub/Hajimi/LXH/_tools/odom_imu.py \
    > /tmp/m2_odom.log 2>&1 < /dev/null &
sleep 6
tail -3 /tmp/m2_odom.log 2>/dev/null | sed 's/^/  /'
echo "  odom_imu 进程: $(pgrep -fc 'odom_imu.py' 2>/dev/null || echo 0)"

echo ""
echo "════════ 4. 等 odom→base_footprint 建立 ════════"
for i in 1 2 3 4 5 6 7 8; do
    if timeout 5 rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -q Translation; then
        echo "  OK  odom -> base_footprint 已通"
        break
    fi
    sleep 2
done
echo "  base_footprint -> laser_link: $(timeout 5 rosrun tf tf_echo base_footprint laser_link 2>/dev/null | grep -c Translation) 条"

echo ""
echo "════════ 5. 起 gmapping ════════"
GM=$WS/src/carebot_nav_kit/config/slam/gmapping_params.yaml
echo "  参数文件: $GM  $([ -f "$GM" ] && echo 存在 || echo 缺失)"
setsid nohup roslaunch "$WS/src/carebot_nav_kit/launch/mapping.launch" \
    slam:=gmapping rviz:=false teleop:=false start_laser_filter:=true \
    > /tmp/m2_gmapping.log 2>&1 < /dev/null &
sleep 18

echo "  gmapping 进程: $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"
echo ""
echo "  日志里的关键参数（应看到 ±4 / particles 50 / delta 0.025）:"
grep -oE '\-xmin -?[0-9.]+|\-xmax -?[0-9.]+|\-ymin -?[0-9.]+|\-ymax -?[0-9.]+|\-particles [0-9]+|\-delta [0-9.]+|\-maxUrange [0-9.]+' \
    /tmp/m2_gmapping.log 2>/dev/null | head -8 | sed 's/^/    /'
echo ""
echo "  错误行（若有）:"
grep -iE 'ERROR|Failed|not found|died' /tmp/m2_gmapping.log 2>/dev/null | head -5 | sed 's/^/    /' || echo "    (无)"

echo ""
echo "════════ 6. 地图状态 ════════"
sleep 3
timeout 40 python /home/ub/Hajimi/LXH/_tools/map_info.py 2>&1 | head -8 | sed 's/^/  /' \
    || echo "  (map_info.py 不在，跳过)"

echo ""
echo "════════ 7. TF 链路总览 ════════"
for pair in "odom base_footprint" "base_footprint laser_link" "base_footprint IMU_link" "map odom"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation \
        && echo "OK" || echo "--"
done

echo ""
echo "════════ 8. 节点 ════════"
timeout 10 rosnode list 2>/dev/null | sed 's/^/  /'

echo ""
echo "════════ 完成 ════════"
echo "  ⚠️ 机器人**仍未移动**"
echo "  下一步: 探索建图（会真的动）"
