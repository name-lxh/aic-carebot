#!/bin/bash
# ============================================================================
#  deploy_lxh.sh —— 把 LXH 包落到小车（在车上执行）
# ============================================================================
#  ⚠️ 边界（用户明确要求"不要碰系统底层的任何东西"）：
#     · 只在 /home/ub/Hajimi/ 下写东西
#     · **不用 sudo**
#     · **不写 /opt/ros、/etc、/usr**
#     · 唯一的跨目录动作是**读取** /opt/ros/.../toplevel.cmake 建软链
#       （软链建在我们自己目录里，源文件只读不写）
# ============================================================================
set -eu

HOME_DIR=/home/ub
TEAM=$HOME_DIR/Hajimi
LXH=$TEAM/LXH
TGZ=/home/ub/Hajimi-LXH.tar.gz

echo "════════ 0. 前置检查 ════════"
[ -f "$TGZ" ] || { echo "  [FAIL] 找不到 $TGZ"; exit 1; }
echo "  包: $TGZ  $(du -h "$TGZ" | cut -f1)"
echo "  磁盘可用: $(df -h /home | tail -1 | awk '{print $4}')"

# ROS 版本（只检测，不修改）
DISTRO=""
for d in kinetic melodic noetic; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then DISTRO=$d; break; fi
done
[ -n "$DISTRO" ] || { echo "  [FAIL] 没找到 ROS"; exit 1; }
echo "  ROS: $DISTRO"

echo ""
echo "════════ 1. 建组目录 /home/ub/Hajimi/LXH ════════"
mkdir -p "$TEAM"
# 如果已存在旧版本，先备份（不删除，安全第一）
if [ -d "$LXH" ]; then
    BK="$TEAM/LXH-backup-$(date +%Y%m%d_%H%M%S).tar.gz"
    echo "  发现已有 LXH，先备份到 $BK"
    tar -czf "$BK" -C "$TEAM" LXH 2>/dev/null || true
fi
echo "  ✔ $LXH"

echo ""
echo "════════ 2. 解压 ════════"
TMP=$(mktemp -d)
tar -xzf "$TGZ" -C "$TMP"
echo "  解压出: $(ls "$TMP")"
# 覆盖式拷贝（保留我们目录里可能已存在的 build/devel）
cp -rf "$TMP/LXH/." "$LXH/"
rm -rf "$TMP"
echo "  ✔ 已落地"

echo ""
echo "════════ 3. 目录结构 ════════"
find "$LXH" -maxdepth 3 -type d | sed "s#$LXH#  LXH#" | sort | head -25

echo ""
echo "════════ 4. 修权限与行尾 ════════"
# 脚本可执行
chmod +x "$LXH"/install.sh 2>/dev/null || true
find "$LXH" -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} \; 2>/dev/null
# 去 BOM / CRLF
fixed=0
while IFS= read -r -d '' f; do
    if grep -qU $'\r' "$f" 2>/dev/null; then
        sed -i 's/\r$//' "$f"; fixed=$((fixed+1))
    fi
    # 去 UTF-8 BOM
    if [ "$(head -c3 "$f" 2>/dev/null | od -An -tx1 | tr -d ' \n')" = "efbbbf" ]; then
        sed -i '1s/^\xEF\xBB\xBF//' "$f"; fixed=$((fixed+1))
    fi
done < <(find "$LXH" -type f \( -name '*.py' -o -name '*.sh' -o -name '*.launch' \
                          -o -name '*.yaml' -o -name '*.xml' -o -name '*.md' \) -print0)
echo "  ✔ $fixed 个文件修正了 CRLF/BOM"

echo ""
echo "════════ 5. ★ 建 catkin toplevel 软链（只写自己目录）════════"
SRC="$LXH/catkin_ws/src"
CM="$SRC/CMakeLists.txt"
TOP="/opt/ros/$DISTRO/share/catkin/cmake/toplevel.cmake"
if [ ! -f "$TOP" ]; then
    echo "  [FAIL] 目标机没有 $TOP"
    echo "         这是 ROS 安装问题，我不去修（不碰系统层）"
    exit 1
fi
if [ -L "$CM" ] || [ -f "$CM" ]; then
    rm -f "$CM"
fi
ln -s "$TOP" "$CM"
echo "  ✔ $CM -> $(readlink "$CM")"

echo ""
echo "════════ 6. 确认没污染系统文件 ════════"
echo "  /opt/ros/$DISTRO/share/catkin/cmake/toplevel.cmake:"
ls -la "$TOP" | sed 's/^/      /'
echo "      大小 $(stat -c %s "$TOP") 字节（应 >0；0 字节说明 catkin 宏加载不了）"
if [ "$(stat -c %s "$TOP")" -lt 100 ]; then
    echo "      [!] 这个文件偏小，可能是被破坏过。**我不改它**，但需要提醒你。"
fi

echo ""
echo "════════ 7. 依赖检查 ════════"
set +u
source "/opt/ros/$DISTRO/setup.bash"
[ -f /home/ub/catkin_ws/devel/setup.bash ] && source /home/ub/catkin_ws/devel/setup.bash
[ -f /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash ] && \
    source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash
set -u
echo "  ROS_PACKAGE_PATH:"
echo "${ROS_PACKAGE_PATH:-}" | tr ':' '\n' | grep . | sed 's/^/      /'
echo ""
miss=0
for pkg in face_detect xf_mic_asr_offline lslidar_driver base_control \
           gmapping amcl move_base laser_filters; do
    if rospack find "$pkg" >/dev/null 2>&1; then
        printf '      OK   %s\n' "$pkg"
    else
        printf '      FAIL %s  (没找到)\n' "$pkg"
        miss=$((miss+1))
    fi
done
echo "  缺失 $miss 个"

echo ""
echo "════════ 8. 完成 ════════"
echo "  包目录: $LXH"
echo "  下一步: bash $LXH/install.sh"
echo "          cd $LXH/catkin_ws && catkin_make"
