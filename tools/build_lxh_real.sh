#!/bin/bash
# ============================================================================
#  build_lxh.sh —— 编译 LXH 工作空间（在车上执行）
# ============================================================================
#  ⚠️⚠️ 为什么不能简单 `source A; source B; catkin_make`
#
#  每个 catkin 工作空间的 `setup.sh` 会调自己的 `_setup_util.py`，它开头写着
#      "reset environment variables by unrolling modifications
#       based on all workspaces in CMAKE_PREFIX_PATH"
#  —— 它会**主动剥掉之前 source 过的工作空间**，只保留"自己 + 基础 ROS"。
#  所以 source 多个工作空间，**只有最后一个会留下**。
#
#  这台车上有两个组委会工作空间：
#      /home/ub/catkin_ws                       （37 个包，主基线）
#      /home/ub/omniWheelCareRobot/rosCode      （robot_voice 等）
#
#  所以本脚本的做法：
#      1. source 主基线（拿到大部分依赖）
#      2. **手工把两个 src 目录都补进 ROS_PACKAGE_PATH**（对抗 unroll）
#      3. catkin_make
#
#  ⚠️ 边界：只在 /home/ub/Hajimi/LXH/ 下生成 build/devel，不碰系统层。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
WS=$LXH/catkin_ws
LOG=/tmp/lxh_build.log

echo "════════ 0. 环境 ════════"
echo "  工作空间: $WS"

DISTRO=""
for d in kinetic melodic noetic; do
    [ -f "/opt/ros/$d/setup.bash" ] && { DISTRO=$d; break; }
done
[ -n "$DISTRO" ] || { echo "  [FAIL] 没找到 ROS"; exit 1; }

set +u
source "/opt/ros/$DISTRO/setup.bash"
# 主基线（尽量多拿依赖）
[ -f /home/ub/catkin_ws/devel/setup.bash ] && \
    source /home/ub/catkin_ws/devel/setup.bash

# ★ 手工补回被 unroll 掉的路径
for d in /home/ub/omniWheelCareRobot/rosCode/src /home/ub/catkin_ws/src; do
    if [ -d "$d" ]; then
        case ":${ROS_PACKAGE_PATH:-}:" in
            *":$d:"*) ;;
            *) ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$d" ;;
        esac
    fi
done
export ROS_PACKAGE_PATH
set -u

echo "  ROS: $DISTRO   Python: $(python --version 2>&1)"
echo "  ROS_PACKAGE_PATH:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep . | sed 's/^/      /'

echo ""
echo "════════ 1. 依赖复查 ════════"
miss=0
for p in face_detect xf_mic_asr_offline lslidar_driver base_control \
         gmapping amcl move_base costmap_2d laser_filters; do
    rospack find "$p" >/dev/null 2>&1 || { printf '  FAIL %s\n' "$p"; miss=$((miss+1)); }
done
echo "  缺失 $miss 个"
[ "$miss" -eq 0 ] || { echo "  [!] 有缺失，继续编译看具体报错"; }

echo ""
echo "════════ 2. 清理旧的 build/devel ════════"
if [ "${1:-}" = "--clean" ]; then
    rm -rf "$WS/build" "$WS/devel"
    echo "  --clean：已删"
else
    echo "  保留（增量编译）"
fi

echo ""
echo "════════ 3. catkin_make ════════"
cd "$WS" || exit 1
t0=$(date +%s)
catkin_make -j2 > "$LOG" 2>&1
RC=$?
t1=$(date +%s)
echo "  返回码 $RC，耗时 $((t1-t0)) 秒"

echo ""
echo "════════ 4. 结果 ════════"
if [ $RC -eq 0 ]; then
    # ⚠️ 判据盯**产物**，不盯日志字符串（catkin 正常也会打印 CATKIN_DEVEL_PREFIX）
    CD=$(grep '^catkin_DIR' "$WS/build/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
    if [ -n "$CD" ] && [ -d "$CD" ]; then
        echo "  [OK] 编译成功（catkin 宏已加载: $CD）"
    else
        echo "  [FAIL] 返回 0 但 catkin 宏没加载 —— 假成功"
        RC=1
    fi
    echo ""
    echo "  产物:"
    [ -f "$WS/devel/setup.bash" ] && echo "      OK devel/setup.bash" \
                                 || echo "      FAIL devel/setup.bash 缺失"
    echo "      消息包:"
    find "$WS/devel" -name 'Control.h' 2>/dev/null | sed 's/^/        /'
    find "$WS/devel/lib/python2.7/dist-packages" -maxdepth 1 -type d \
         -name 'carebot_*' 2>/dev/null | sed 's#.*/#        #'
else
    echo "  [FAIL] 编译失败 —— 错误摘要："
    grep -nE 'error:|Error|CMake Error|fatal error|Invoking "make"|No such file' \
        "$LOG" | head -25 | sed 's/^/      /'
    echo ""
    echo "  完整日志: $LOG"
fi

echo ""
echo "════════ 5. 完成 ════════"
echo "  日志: $LOG"
exit $RC
