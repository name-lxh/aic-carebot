#!/bin/bash
# ============================================================================
#  m3_full_slam.sh —— 一次性干净地把建图栈整套拉起来
# ============================================================================
#  ## 为什么推倒重来
#
#  前面 m1/m2/m2b 是**增量**往上加的，结果越加越乱：
#
#    · `mapping.launch` 内含 bringup.launch，**又起了一遍 lidar**
#      → 和 m1 起的那个抢同一个 /dev/carebot_laser_ls，一个必死
#    · 我 pkill mapping.launch 时，把**活着的那个 lidar 也杀了** → /scan 消失
#    · 跟着 /scan_filtered 空 → gmapping 没数据 → /map 有话题但永远没帧
#    · 还残留 ekf_se / odom_relay / art_imu_node(旧版) 抢 /odom 和串口
#
#  ## 本脚本的做法
#
#  先**全部停掉**（只留 rosmaster），再按固定顺序起一遍：
#
#      1. lidar        → /scan           (10 Hz)
#      2. base(stm32)  → /cmd_vel 接收 + /odom（不可信，不用它的值）
#      3. imu(rosCode 的 art_imu_01) → /imu_data (200 Hz, 航向 0.013°)
#      4. 静态 TF      → base_footprint→base_link→{laser,IMU,camera}
#      5. odom_imu.py  → odom→base_footprint(IMU 航向 + 命令积分平移)
#      6. laser_filter → /scan_filtered
#      7. gmapping     → /map   map→odom
#
#  ⚠️ 用的是**新的 rosrun + rosparam** 方式，不是 mapping.launch ——
#     因为 mapping.launch 会连带 bringup(EKF/静态TF/lidar) 造成重复。
#
#  ⚠️ 本脚本**不发速度**，机器人不动。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 0. 全停（只留 rosmaster）════════"
pkill -x slam_gmapping      2>/dev/null
pkill -x ekf_se             2>/dev/null
pkill -x laser_filter       2>/dev/null
pkill -x art_imu_node       2>/dev/null
pkill -x art_imu_01_node    2>/dev/null
pkill -x lslidar_driver_node 2>/dev/null
pkill -x stm32_node         2>/dev/null
pkill -f 'odom_relay'       2>/dev/null
pkill -f 'odom_imu.py'      2>/dev/null
pkill -f 'static_transform_publisher' 2>/dev/null
pkill -f 'tf_echo'          2>/dev/null
pkill -f 'roslaunch'        2>/dev/null
sleep 5
echo "  残留: gmapping=$(pgrep -xc slam_gmapping 2>/dev/null || echo 0) lidar=$(pgrep -xc lslidar_driver_node 2>/dev/null || echo 0) imu=$(pgrep -xc art_imu_01_node 2>/dev/null || echo 0)"
echo "  rosmaster: $(pgrep -xc rosmaster 2>/dev/null || echo 0)"

if ! pgrep -x rosmaster >/dev/null 2>&1; then
    echo "  重起 roscore"
    setsid nohup roscore > /tmp/m3_roscore.log 2>&1 < /dev/null &
    sleep 8
fi

echo ""
echo "════════ 1. 雷达 ════════"
setsid nohup roslaunch lslidar_driver lslidar_serial.launch \
    > /tmp/m3_lidar.log 2>&1 < /dev/null &
sleep 12
SCAN_HZ=$(timeout 12 rostopic hz /scan 2>&1 | grep -m1 average || echo '')
echo "  /scan: ${SCAN_HZ:-★无数据}"

echo ""
echo "════════ 2. 底盘 ════════"
setsid nohup roslaunch base_control stm32.launch \
    > /tmp/m3_base.log 2>&1 < /dev/null &
sleep 10
echo "  /odom: $(timeout 10 rostopic hz /odom 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 3. IMU（rosCode 的 YESENSE 版 @460800）════════"
setsid nohup roslaunch "$WS/src/carebot_nav_kit/launch/include/imu.launch" \
    > /tmp/m3_imu.log 2>&1 < /dev/null &
sleep 14
echo "  /imu_data: $(timeout 12 rostopic hz /imu_data 2>&1 | grep -m1 average || echo '★无数据')"
echo "  进程: $(pgrep -xc art_imu_01_node 2>/dev/null || echo 0) 个 art_imu_01_node"

echo ""
echo "════════ 4. 静态 TF ════════"
for spec in "0 0 0.06 0 0 0 base_footprint base_link" \
            "0 0 0.21 0 0 0 base_link laser_link" \
            "0.125 0 0.11 0 0 0 base_link IMU_link"; do
    setsid nohup rosrun tf static_transform_publisher $spec 50 \
        > /dev/null 2>&1 < /dev/null &
    sleep 1
done
sleep 2
echo "  base_footprint→laser_link: $(timeout 5 rosrun tf tf_echo base_footprint laser_link 2>/dev/null | grep -c Translation)"

echo ""
echo "════════ 5. 干净 odom 源 ════════"
setsid nohup python "$LXH/_tools/odom_imu.py" \
    > /tmp/m3_odom.log 2>&1 < /dev/null &
sleep 8
echo "  odom→base_footprint: $(timeout 6 rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -c Translation)"
tail -2 /tmp/m3_odom.log 2>/dev/null | sed 's/^/    /'

echo ""
echo "════════ 6. 激光滤波 ════════"
setsid nohup roslaunch "$WS/src/carebot_nav_kit/launch/include/laser_filter.launch" \
    > /tmp/m3_filter.log 2>&1 < /dev/null &
sleep 10
echo "  /scan_filtered: $(timeout 12 rostopic hz /scan_filtered 2>&1 | grep -m1 average || echo '★无数据')"

echo ""
echo "════════ 7. gmapping ════════"
GP=$WS/src/carebot_nav_kit/config/slam/gmapping_params.yaml
rosparam load "$GP" /slam_gmapping 2>/dev/null
echo "  参数已载入: xmin=$(rosparam get /slam_gmapping/xmin 2>/dev/null) particles=$(rosparam get /slam_gmapping/particles 2>/dev/null) delta=$(rosparam get /slam_gmapping/delta 2>/dev/null)"
setsid nohup rosrun gmapping slam_gmapping __name:=slam_gmapping scan:=/scan_filtered \
    > /tmp/m3_gmapping.log 2>&1 < /dev/null &
sleep 15
echo "  gmapping 进程: $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"
echo "  日志参数:"
grep -oE '\-xmin -?[0-9.]+|\-particles [0-9]+|\-delta [0-9.]+|\-maxUrange [0-9.]+' \
    /tmp/m3_gmapping.log 2>/dev/null | head -5 | sed 's/^/    /'

echo ""
echo "════════ 8. ★ /map 有没有真的收到数据 ════════"
sleep 5
timeout 45 python /home/ub/Hajimi/LXH/_tools/map_info.py 2>&1 | head -10 | sed 's/^/  /'

echo ""
echo "════════ 9. 完整 TF 链 ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link" "map laser_link"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo "--"
done

echo ""
echo "════════ 10. 节点清单（每个只应有一个）════════"
timeout 10 rosnode list 2>/dev/null | grep -vE 'tf_echo|_tf_echo' | sed 's/^/  /'

echo ""
echo "════════ 完成 ════════"
echo "  ⚠️ 机器人**未移动**"
