#!/bin/bash
# ============================================================================
#  install.sh —— LXH 包在工控机上的落地脚本
# ============================================================================
#  用途：解压之后跑一次，把工作空间骨架补齐、权限理顺、环境自检。
#
#  ⚠️ 这个脚本**只动 LXH 自己的目录**，不会碰：
#       · /home/ub/catkin_ws        （组委会工作空间）
#       · /home/ub/ 根目录下的任何东西
#       · 其他组的目录（talos_scripts 等）
#     也**不需要 sudo**。
#
#  为什么软链要在这里建：Windows 上打包时建不了 Unix 软链，
#  而且软链本来就该指向**目标机**的路径
#  （工控机是 kinetic，虚拟机上要改成 melodic）。
# ============================================================================
set -u

LXH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "════════ LXH 安装自检 ════════"
echo "  包目录: $LXH"
echo "  用户:   $(whoami)"
echo ""

# ---- 0. 安全检查：确认我们在自己的目录里 ----
case "$LXH" in
    */Hajimi/LXH) : ;;
    *)
        echo "  ⚠️ 当前不在 .../Hajimi/LXH 下（实际：$LXH）"
        echo "     如果只是解压到别处看看没问题，继续即可。"
        ;;
esac

# ---- 1. 探测 ROS 发行版 ----
DISTRO=""
for d in kinetic melodic noetic; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
        DISTRO="$d"
        break
    fi
done
if [ -z "$DISTRO" ]; then
    echo "  ✘ 找不到 /opt/ros/{kinetic,melodic,noetic} —— ROS 没装？"
    exit 1
fi
echo "  探测到 ROS: $DISTRO"

# ---- 2. 建 catkin toplevel 软链 ----
SRC="$LXH/catkin_ws/src"
CM="$SRC/CMakeLists.txt"
TOP="/opt/ros/$DISTRO/share/catkin/cmake/toplevel.cmake"
if [ -f "$TOP" ]; then
    if [ -L "$CM" ] || [ -f "$CM" ]; then
        rm -f "$CM"
    fi
    ln -s "$TOP" "$CM"
    echo "  ✔ src/CMakeLists.txt -> $TOP"
else
    echo "  ✘ 找不到 $TOP"
fi

# ---- 3. 权限：ROS 节点和脚本必须可执行 ----
n=0
for d in "$SRC"/*/nodes "$SRC"/*/scripts "$LXH/_tools"; do
    [ -d "$d" ] || continue
    while IFS= read -r -d '' f; do
        chmod +x "$f" 2>/dev/null && n=$((n+1))
    done < <(find "$d" -maxdepth 1 -type f \( -name '*.py' -o -name '*.sh' \) -print0 2>/dev/null)
done
echo "  ✔ 已给 $n 个文件加可执行位"

# ---- 4. 清掉可能被 Windows 带过来的 BOM / CRLF ----
b=0
while IFS= read -r -d '' f; do
    # BOM
    if head -c 3 "$f" 2>/dev/null | grep -q $'\xef\xbb\xbf'; then
        sed -i '1s/^\xef\xbb\xbf//' "$f" && b=$((b+1))
    fi
    # CRLF（只处理文本文件）
    if file "$f" 2>/dev/null | grep -q CRLF; then
        sed -i 's/\r$//' "$f" && b=$((b+1))
    fi
done < <(find "$LXH" -type f \( -name '*.py' -o -name '*.sh' \
         -o -name '*.launch' -o -name '*.yaml' -o -name '*.xml' \
         -o -name '*.md' \) -print0 2>/dev/null)
echo "  ✔ 修正 $b 个文件的 BOM/CRLF"

# ---- 5. 内容清点 ----
echo ""
echo "  ── 我们的包 ──"
for p in "$SRC"/*/; do
    [ -d "$p" ] || continue
    nm=$(basename "$p")
    [ -f "$p/package.xml" ] && echo "      $nm"
done

echo ""
echo "  ── 组委会的包（不在本包内，需另行确认已装）──"
# ⚠️⚠️ source ROS 的 setup.bash 前**必须临时关掉 `set -u`**。
#
# 踩过：脚本开头有 `set -u`（未定义变量即报错退出），
# 而 `/opt/ros/*/setup.bash` 内部会引用一些**可能未定义**的变量
# （ROS_DISTRO、CMAKE_PREFIX_PATH 等），
# 于是 source 到一半整个脚本**静默退出**（exit 1），
# 后面的检查一句都没跑 —— 看起来像"脚本坏了"，其实是 shell 选项冲突。
#
# 做法：source 期间 `set +u`，之后恢复。
set +u
source "/opt/ros/$DISTRO/setup.bash" 2>/dev/null
if [ -f /home/ub/catkin_ws/devel/setup.bash ]; then
    source /home/ub/catkin_ws/devel/setup.bash 2>/dev/null
    echo "      （已加载 /home/ub/catkin_ws/devel）"
fi
set -u
miss=0
# ⚠️⚠️ 这里写的是**包名（package.xml 里的 <name>）**，不是目录名！
#
# 踩过：`face_recognition` 是**目录名**，包名其实是 `face_detect`
# （`<name>face_detect</name>`）。用目录名去 `rospack find` 会报
# "not found"，看起来像"依赖没装"，其实好好的。
#
# 本项目已知的 3 处"目录名 ≠ 包名"：
#     face_recognition      → face_detect
#     ros_astra_camera      → astra_camera
#     android_communication → android_app_communication
#
# 重新算依赖清单：_tools/compute_vendor_closure.py
for pkg in lslidar_driver lslidar_msgs base_control face_detect \
           fdilink_ahrs xf_mic_asr_offline laser_filters; do
    if rospack find "$pkg" >/dev/null 2>&1; then
        printf '      ✔ %s\n' "$pkg"
    else
        printf '      ✘ %s  ★ 没找到\n' "$pkg"
        miss=$((miss+1))
    fi
done
if [ "$miss" -gt 0 ]; then
    echo ""
    echo "  ⚠️ 有 $miss 个依赖包没找到。多半是没 source 组委会工作空间，试试："
    echo "       source /home/ub/catkin_ws/devel/setup.bash"
fi

# ---- 6. Python 版本 & 语法体检 ----
echo ""
echo "  ── Python 版本 ──"
python --version 2>&1 | sed 's/^/      /'

if command -v python2 >/dev/null 2>&1; then
    bad=0
    while IFS= read -r -d '' f; do
        python2 -c "import ast,sys; ast.parse(open(sys.argv[1],'rb').read())" \
            "$f" 2>/dev/null || { printf '      ✘ 语法错: %s\n' "$f"; bad=$((bad+1)); }
    done < <(find "$SRC" -name '*.py' -print0 2>/dev/null)
    if [ "$bad" -eq 0 ]; then
        echo "      ✔ 全部 .py 通过 Py2 语法检查"
    else
        echo "      ★ $bad 个文件有问题"
    fi
fi

# ---- 7. 磁盘 ----
echo ""
echo "  ── 磁盘 ──"
df -h "$LXH" | tail -1 | sed 's/^/      /'

echo ""
echo "════════ 完成 ════════"
echo "  下一步（要编译的话）："
echo "      source /opt/ros/$DISTRO/setup.bash"
echo "      source /home/ub/catkin_ws/devel/setup.bash   # 先组委会"
echo "      cd $LXH/catkin_ws && catkin_make"
echo "      source devel/setup.bash                      # 后我们自己"
echo ""
echo "  ⚠️ source 顺序不能反：后 source 的覆盖先 source 的。"
echo "     先组委会、后我们，同名包才会以我们的为准。"
echo ""
