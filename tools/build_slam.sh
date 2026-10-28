#!/bin/bash
# ============================================================================
#  build_slam.sh —— 起建图（gmapping）所需的完整链路
# ============================================================================
#  ⚠️ 现状（实测）：
#     · TF 树**完全空的** —— 没有任何节点广播 odom->base_footprint
#       （stm32.launch 里 is_pub_odom_tf=false）
#     · /odom 数据不可信（绝对编码器值 + 静止漂 44°）
#     → 所以要先自己起一个干净的 odom 源，再补静态 TF，最后才是 gmapping
#
#  链路：
#     roscore
#     ├── lslidar_driver         → /scan
#     ├── art_imu_01_node        → /imu_data
#     ├── stm32_node             → /odom /cmd_vel （底盘，但不广播 TF）
#     ├── static_tf              → base_footprint->laser_link (0,0,0.21)
#     ├── static_tf              → base_footprint->imu_link   (0.125,0,0.11)
#     ├── odom_imu.py            → odom->base_footprint （用 IMU 航向 + 平滑平移）
#     └── slam_gmapping          → /map  map->odom
# ============================================================================
set -u

ROS_IP_OURS=192.168.43.233
export ROS_IP="$ROS_IP_OURS"
export ROS_MASTER_URI="http://${ROS_IP_OURS}:11399"

echo "════════ 0. 环境 ════════"
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
echo "  ROS_IP=$ROS_IP"

echo ""
echo "════════ 1. 清理残留 ════════"
for p in roslaunch rosmaster roscore lslidar stm32 art_imu odom_imu \
         slam_gmapping static_transform; do
    pkill -f "$p" 2>/dev/null
done
sleep 3
rm -rf /home/ub/.ros/log/* 2>/dev/null
echo "  已清理"

echo ""
echo "════════ 2. roscore ════════"
roscore > /tmp/slam_roscore.log 2>&1 &
sleep 8
pgrep -x rosmaster >/dev/null && echo "  ✔ roscore 在跑" || {
    echo "  ✘ roscore 没起来"; tail -10 /tmp/slam_roscore.log; exit 1; }

echo ""
echo "════════ 3. 雷达 ════════"
roslaunch lslidar_driver lslidar_serial.launch > /tmp/slam_lidar.log 2>&1 &
sleep 10
echo "  /scan: $(timeout 8 rostopic hz /scan 2>&1 | grep -m1 average || echo '无数据')"

echo ""
echo "════════ 4. IMU ════════"
roslaunch art_imu_01 imu_serial.launch > /tmp/slam_imu.log 2>&1 &
sleep 10
echo "  /imu_data: $(timeout 8 rostopic hz /imu_data 2>&1 | grep -m1 average || echo '无数据')"

echo ""
echo "════════ 5. 底盘（只取 /odom，不发指令）════════"
roslaunch base_control stm32.launch > /tmp/slam_base.log 2>&1 &
sleep 8
echo "  /odom: $(timeout 8 rostopic hz /odom 2>&1 | grep -m1 average || echo '无数据')"

echo ""
echo "════════ 6. ★ 静态 TF（补 base_footprint -> laser_link / imu_link）════════"
# 参数取自组委会 carebot_navigation/launch/car_nav.launch（与我们的代码一致）
rosrun tf static_transform_publisher 0 0 0.21 0 0 0 \
    base_link laser_link 100 > /tmp/slam_tf1.log 2>&1 &
sleep 2
rosrun tf static_transform_publisher 0 0 0 0 0 0 \
    base_footprint base_link 100 > /tmp/slam_tf2.log 2>&1 &
sleep 2
rosrun tf static_transform_publisher 0.125 0 0.11 0 0 0 \
    base_link IMU_link 100 > /tmp/slam_tf3.log 2>&1 &
sleep 2
echo "  已起 3 个静态 TF（base_footprint->base_link->laser_link/IMU_link）"

echo ""
echo "════════ 7. ★ 干净的 odom 源 ════════"
python /tmp/odom_imu.py > /tmp/slam_odomimu.log 2>&1 &
sleep 8
echo "  日志：$(tail -3 /tmp/slam_odomimu.log | tr '\n' ' ')"

echo ""
echo "════════ 8. ★ 检查 TF 链是否通了 ════════"
for pair in "base_footprint laser_link" "odom base_footprint"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    if timeout 6 rosrun tf tf_echo "$1" "$2" 2>&1 | grep -q Translation; then
        echo "✔ 通"
    else
        echo "✘ 不通"
    fi
done

echo ""
echo "════════ 9. ★ 起 gmapping ════════"
cat > /tmp/gmapping_params.yaml <<'YEOF'
base_frame: base_footprint
odom_frame: odom
map_frame: map
map_update_interval: 2.0
maxUrange: 6.0
maxRange: 8.0
sigma: 0.05
kernelSize: 1
lstep: 0.05
astep: 0.05
iterations: 5
lsigma: 0.075
ogain: 3.0
lskip: 0
minimumScore: 30
srr: 0.1
srt: 0.2
str: 0.1
stt: 0.2
linearUpdate: 0.15
angularUpdate: 0.25
temporalUpdate: 1.0
resampleThreshold: 0.5
particles: 50
xmin: -5.0
ymin: -5.0
xmax: 5.0
ymax: 5.0
delta: 0.05
llsamplerange: 0.01
llsamplestep: 0.01
lasamplerange: 0.005
lasamplestep: 0.005
YEOF
rosrun gmapping slam_gmapping \
    _base_frame:=base_footprint _odom_frame:=odom _map_frame:=map \
    __name:=slam_gmapping > /tmp/slam_gmapping.log 2>&1 &
sleep 12

echo "  节点："
timeout 8 rosnode list 2>/dev/null | sed 's/^/      /'
echo "  /map 有没有："
if timeout 10 rostopic list 2>/dev/null | grep -qx '/map'; then
    echo "      ✔ /map 话题在"
    timeout 15 python - <<'PYEOF' 2>&1 | sed 's/^/      /'
from __future__ import print_function
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('map_check', anonymous=True)
try:
    m = rospy.wait_for_message('/map', OccupancyGrid, timeout=12)
    print('地图 %dx%d @%.2f m' % (m.info.width, m.info.height,
                                  m.info.resolution))
    print('origin (%.2f, %.2f)' % (m.info.origin.position.x,
                                   m.info.origin.position.y))
    d = m.data
    print('已知格 %d / %d' % (sum(1 for v in d if v >= 0), len(d)))
except Exception as e:
    print('读不到 /map: %r' % e)
PYEOF
else
    echo "      ✘ 没有 /map"
    tail -20 /tmp/slam_gmapping.log | sed 's/^/      /'
fi

echo ""
echo "════════ 10. 完成 ════════"
echo "  gmapping 日志：/tmp/slam_gmapping.log"
echo "  ⚠️ 此时机器人仍未移动 —— 建图需要一边走一边建"
