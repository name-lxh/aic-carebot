#!/bin/bash
# ============================================================================
#  real_robot_recon.sh —— 实地测试前的只读侦察
# ============================================================================
#  ⚠️ 这个脚本**只读**：不启动节点、不发运动指令、不写参数。
#     实地测试的第一步必须是"看清现状"，尤其是别人的机器人上
#     已经跑着什么、有没有现成的地图和导航栈。
# ============================================================================
set -u

echo "════════ 0. 基本环境 ════════"
echo "  时间:   $(date '+%F %T')"
echo "  主机:   $(hostname)"
echo "  用户:   $(whoami)"
echo "  内核:   $(uname -r)"
echo "  负载:   $(uptime | sed 's/.*load average: //')"
echo "  磁盘:   $(df -h /home | tail -1 | awk '{print $4}') 可用"

echo ""
echo "════════ 1. 有没有 ROS 在跑 ════════"
if pgrep -x rosmaster >/dev/null 2>&1 || pgrep -x roscore >/dev/null 2>&1; then
    echo "  ★ 已有 roscore/rosmaster 在跑（PID: $(pgrep -x rosmaster | tr '\n' ' '))"
    echo "    ⚠️ 说明**别人可能正在用**，先别动"
else
    echo "  · 没有 roscore 在跑"
fi
echo "  roslaunch 进程: $(pgrep -fc roslaunch 2>/dev/null || echo 0)"
echo "  其他 ros 节点进程数: $(pgrep -fc 'ros|node' 2>/dev/null || echo 0)"

echo ""
echo "════════ 2. ROS 环境 ════════"
set +u
source /opt/ros/kinetic/setup.bash
[ -f /home/ub/catkin_ws/devel/setup.bash ] && source /home/ub/catkin_ws/devel/setup.bash
set -u
echo "  ROS_DISTRO=${ROS_DISTRO:-?}"
echo "  ROS_MASTER_URI=${ROS_MASTER_URI:-（未设）}"
echo "  ROS_IP=${ROS_IP:-（未设）}  ROS_HOSTNAME=${ROS_HOSTNAME:-（未设）}"
echo "  ROS_PACKAGE_PATH:"
echo "${ROS_PACKAGE_PATH:-}" | tr ':' '\n' | grep . | sed 's/^/      /'

echo ""
echo "════════ 3. ★ 组委会的包（这台机器上有什么）════════"
echo "  --- carebot_* 系列 ---"
for p in carebot_bringup carebot_navigation carebot_tasks carebot_msgs \
         carebot_description carebot_urdf carebot_auto_gmapping \
         carebot_calibration carebot_follower carebot_velocity_smoother \
         carebot_testsuite; do
    d=$(rospack find "$p" 2>/dev/null)
    [ -n "$d" ] && printf '      ✔ %-28s %s\n' "$p" "$d"
done
echo "  --- 硬件驱动 ---"
for p in base_control lslidar_driver lslidar_msgs fdilink_ahrs \
         astra_camera astra_launch face_detect xf_mic_asr_offline \
         laser_filters robot_localization; do
    d=$(rospack find "$p" 2>/dev/null)
    [ -n "$d" ] && printf '      ✔ %-28s %s\n' "$p" "$d"
done

echo ""
echo "════════ 4. ★ 现有地图 ════════"
echo "  --- 找 .yaml 地图 ---"
find /home/ub -maxdepth 4 -name '*.yaml' 2>/dev/null \
    | xargs grep -l 'image:' 2>/dev/null | head -10 | sed 's/^/      /'
echo "  --- 根目录下的地图文件 ---"
ls -la /home/ub/*.pgm /home/ub/*.yaml 2>/dev/null | sed 's/^/      /'

echo ""
echo "════════ 5. ★ 串口设备（底盘/雷达/IMU 在哪）════════"
ls -la /dev/ttyUSB* /dev/ttyACM* /dev/stm32 /dev/carebot_laser_ls \
       /dev/laser /dev/fdilink_ahrs /dev/art_imu /dev/IOT_net 2>/dev/null \
    | sed 's/^/      /'
echo "  （没有的话说明设备没插好，或者 udev 规则没生效）"

echo ""
echo "════════ 6. 网络 ════════"
ip -4 addr show 2>/dev/null | grep -E 'inet ' | grep -v 127.0.0.1 | sed 's/^/      /'

echo ""
echo "════════ 7. ★ 组委会的 launch 文件（导航怎么起）════════"
NB=$(rospack find carebot_bringup 2>/dev/null)
NN=$(rospack find carebot_navigation 2>/dev/null)
if [ -n "$NB" ]; then
    echo "  carebot_bringup/launch:"
    ls "$NB/launch" 2>/dev/null | sed 's/^/      /'
fi
if [ -n "$NN" ]; then
    echo "  carebot_navigation/launch:"
    ls "$NN/launch" 2>/dev/null | sed 's/^/      /'
fi

echo ""
echo "════════ 8. 组委会的启动脚本 / 说明 ════════"
ls -la /home/ub/*.sh 2>/dev/null | sed 's/^/      /'
ls /home/ub/Documents 2>/dev/null | head -10 | sed 's/^/      /'

echo ""
echo "════════ 9. 电池/电机状态（如果有相关话题或设备）════════"
ls /dev/tty* 2>/dev/null | head -20 | sed 's/^/      /'

echo ""
echo "════════ 侦察完成（未做任何修改）════════"
