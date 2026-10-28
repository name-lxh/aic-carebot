#!/bin/bash
# ============================================================================
#  prep_mapping.sh —— 建图前的准备检查（**只读，不启动任何节点**）
# ============================================================================
#  用户要求："部署完成后先调试再准备建图"
#  这一步只做"能不能建图"的体检：
#     1. 硬件串口（底盘 / 雷达 / IMU）在不在、权限对不对
#     2. udev 规则是否匹配（这决定设备名是否稳定）
#     3. 我们的 mapping.launch 依赖的包是否齐
#     4. 地图输出目录是否有写权限
#     5. 组委会基线工作空间是否完好（我们 overlay 在它上面）
#
#  ⚠️ 不碰系统层：不写 /etc/udev、不改 /opt/ros、不用 sudo。
#     只**读取**现状，如果有问题就报告，交给人决定。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws

echo "════════ 0. 环境 ════════"
set +u
source /opt/ros/kinetic/setup.bash
[ -f /home/ub/catkin_ws/devel/setup.bash ] && source /home/ub/catkin_ws/devel/setup.bash
source "$WS/devel/setup.bash" 2>/dev/null
for d in /home/ub/omniWheelCareRobot/rosCode/src /home/ub/catkin_ws/src; do
    [ -d "$d" ] || continue
    case ":${ROS_PACKAGE_PATH:-}:" in
        *":$d:"*) ;;
        *) ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$d" ;;
    esac
done
export ROS_PACKAGE_PATH
set -u
echo "  ROS_DISTRO=${ROS_DISTRO:-?}  Python: $(python --version 2>&1)"
echo "  负载: $(uptime | sed 's/.*load average: //')"
echo "  磁盘: $(df -h /home | tail -1 | awk '{print $4}') 可用"

echo ""
echo "════════ 1. ★ 硬件串口 ════════"
echo "  --- 期望的固定链接（udev）---"
for d in /dev/stm32 /dev/carebot_laser_ls /dev/art_imu /dev/fdilink_ahrs; do
    if [ -e "$d" ]; then
        tgt=$(readlink -f "$d" 2>/dev/null)
        perm=$(stat -c '%A %U:%G' "$d" 2>/dev/null)
        printf '      OK   %-24s -> %-14s %s\n' "$d" "$tgt" "$perm"
        [ -r "$d" ] && [ -w "$d" ] && printf '           %-24s 可读写\n' "" \
                                    || printf '           %-24s [!] 权限不足\n' ""
    else
        printf '      --   %-24s 不存在\n' "$d"
    fi
done

echo ""
echo "  --- 实际存在的 USB/ACM 设备 ---"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/      /' \
    || echo "      （没有 ttyUSB* / ttyACM*）"

echo ""
echo "  --- 当前用户所在的串口组 ---"
id | tr ',' '\n' | grep -iE 'dialout|tty' | sed 's/^/      /' || echo "      [!] 不在 dialout 组"

echo ""
echo "════════ 2. udev 规则（只读查看）════════"
echo "  与 carebot/stm32/lidar/imu/astra 相关的规则文件:"
for f in /etc/udev/rules.d/*; do
    [ -f "$f" ] || continue
    if grep -qiE 'stm32|lidar|ls01|art_imu|fdilink|astra|rplidar|carebot' "$f" 2>/dev/null; then
        printf '      %s\n' "$f"
    fi
done

echo ""
echo "  规则里定义的名字（抽样）:"
grep -rhoE 'SYMLINK\+?=\\"?[A-Za-z0-9_./-]+' /etc/udev/rules.d/ 2>/dev/null \
    | sed 's/.*=//; s/"//g' | sort -u | head -20 | sed 's/^/      /'

echo ""
echo "════════ 3. ★ 建图所需的包 ════════"
miss=0
for p in gmapping map_server laser_filters lslidar_driver base_control \
         art_imu_01 carebot_nav_kit; do
    r=$(rospack find "$p" 2>/dev/null)
    if [ -n "$r" ]; then
        printf '      OK   %-18s %s\n' "$p" "$r"
    else
        printf '      FAIL %-18s 没找到\n' "$p"
        miss=$((miss+1))
    fi
done
echo "      缺失 $miss 个"

echo ""
echo "════════ 4. 我们的建图 launch 会启动什么 ════════"
L=$WS/src/carebot_nav_kit/launch/mapping.launch
if [ -f "$L" ]; then
    echo "  $L"
    echo "  包含的 include:"
    grep -oE 'file="\$\(find [^)]*\)[^"]*"' "$L" | sed 's/file="/      /; s/"$//'
    echo "  启动的 node:"
    grep -oE 'type="[^"]*"[^>]*name="[^"]*"' "$L" | head -10 | sed 's/^/      /'
    echo ""
    echo "  roslaunch --nodes 解析结果: $(timeout 40 roslaunch --nodes "$L" 2>/dev/null | wc -l) 个节点"
else
    echo "  [!] 找不到 $L"
fi

echo ""
echo "════════ 5. ★ 地图输出目录 ════════"
for d in "$WS/src/carebot_nav_kit/maps" "$WS/src/carebot_nav_kit/config"; do
    if [ -d "$d" ]; then
        n=$(ls "$d" 2>/dev/null | wc -l)
        w=$([ -w "$d" ] && echo "可写" || echo "[!] 不可写")
        printf '      OK   %-50s %2d 项  %s\n' "${d#$LXH/}" "$n" "$w"
    else
        printf '      --   %-50s 目录不存在\n' "${d#$LXH/}"
    fi
done
echo ""
echo "  已有地图文件:"
find "$WS/src/carebot_nav_kit" -name '*.yaml' -path '*maps*' 2>/dev/null | sed 's/^/      /' \
    || echo "      （还没有地图，建图后生成）"

echo ""
echo "════════ 6. 组委会基线完好性 ════════"
echo "  /home/ub/catkin_ws:"
echo "      src 包数: $(ls /home/ub/catkin_ws/src 2>/dev/null | wc -l)"
echo "      devel/setup.bash: $([ -f /home/ub/catkin_ws/devel/setup.bash ] && echo '在' || echo '缺失')"
echo "  /home/ub/omniWheelCareRobot/rosCode:"
echo "      src 包数: $(ls /home/ub/omniWheelCareRobot/rosCode/src 2>/dev/null | wc -l)"
echo "      devel/setup.bash: $([ -f /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash ] && echo '在' || echo '缺失')"

echo ""
echo "════════ 7. 系统层未被我们改动（核对）════════"
echo "  /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake:"
stat -c '      %n  %s 字节  属主 %U:%G  改于 %y' \
    /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake 2>/dev/null
echo "  （我们只**读取**它建软链，没有写它）"
echo ""
echo "  我们自己的目录属主:"
stat -c '      %n  %U:%G' "$LXH" 2>/dev/null

echo ""
echo "════════ 结论 ════════"
if [ "$miss" -eq 0 ]; then
    echo "  [OK] 建图所需的包齐全"
else
    echo "  [!] 缺 $miss 个包，建图前要解决"
fi
echo "  ⚠️ 以上只做检查 —— **没有启动任何节点、没有移动机器人**"
