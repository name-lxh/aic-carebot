#!/bin/bash
# ============================================================================
#  m20_remap.sh —— 用**修好的 odom** 重扫地图
# ============================================================================
#  ## 为什么必须重扫
#
#  10-19 建的三张地图（map1 / map3 / merged / clean）**全部是基于有 bug 的
#  odom_imu 建的**。那个 bug 让 odom 记录的位移方向整体旋转 yaw0（本车 -97.5°），
#  而 gmapping 把 odom 当运动先验 —— 先验方向错 97.5°，建出来的地图必然是歪的。
#
#  这解释了：
#    · 地图里那条"斜向支路"（很可能是 bug 造成的扭曲，不是真实结构）
#    · 房间内的斜向条纹伪影
#    · AMCL 怎么调都对不上（实测最佳/次优得分只差 1.5%，完全无解）
#    · 地图看起来"只有一个房间"，而实际场地是 3x3 四个房间
#
#  ## 这一轮
#
#  机器人已被用户搬到**起点**（地图开口处），odom 已归零。
#  起全新 gmapping，用/good odom，先做慢转扫描把四周吃进来。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

echo "════════ 1. 停掉旧的地图/导航节点 ════════"
for p in slam_gmapping amcl move_base map_server; do pkill -x "$p" 2>/dev/null; done
sleep 4
echo "  已停"

echo ""
echo "════════ 2. 确认基础栈 ════════"
for t in /scan /scan_filtered /imu_data /odom_clean; do
    printf '  %-16s ' "$t"
    hz=$(timeout 8 rostopic hz "$t" 2>&1 | grep -m1 average || echo '')
    echo "${hz:-★无数据}"
done
printf '  %-16s ' "odom->base TF"
timeout 5 rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -q Translation && echo OK || echo FAIL

echo ""
echo "════════ 3. 起全新 gmapping ════════"
GP=$WS/src/carebot_nav_kit/config/slam/gmapping_params.yaml
rosparam load "$GP" /slam_gmapping 2>/dev/null
echo "  xmin=$(rosparam get /slam_gmapping/xmin 2>/dev/null) particles=$(rosparam get /slam_gmapping/particles 2>/dev/null) delta=$(rosparam get /slam_gmapping/delta 2>/dev/null) maxUrange=$(rosparam get /slam_gmapping/maxUrange 2>/dev/null)"
setsid nohup rosrun gmapping slam_gmapping __name:=slam_gmapping scan:=/scan_filtered \
    > /tmp/m20_gmapping.log 2>&1 < /dev/null &
sleep 18
echo "  gmapping 进程: $(pgrep -xc slam_gmapping 2>/dev/null || echo 0)"
printf '  /map: '
timeout 12 rostopic hz /map 2>&1 | grep -m1 average || echo '★无数据'

echo ""
echo "════════ 4. TF 链 ════════"
for pair in "map odom" "odom base_footprint" "base_footprint laser_link"; do
    set -- $pair
    printf '  %-28s ' "$1 -> $2"
    timeout 5 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation && echo OK || echo '--'
done

echo ""
echo "  准备就绪 —— 下一步跑慢转扫描"
