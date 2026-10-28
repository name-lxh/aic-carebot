#!/bin/bash
# ============================================================================
#  check_hardware.sh — 上车前硬件层检查（不依赖 ROS）
# ============================================================================
#  与 nav_check.py 的分工：
#    check_hardware.sh  —— 查"设备节点在不在、权限对不对"，**不需要 roscore**
#    nav_check.py       —— 查"ROS 图通不通、数据有没有",  需要 roscore
#
#  排队用机器人的时候，先跑这个，能在 10 秒内排除掉大部分"设备没插好"
#  这类浪费时间的低级问题。
#
#  用法：
#      rosrun carebot_nav_kit check_hardware.sh
#      bash check_hardware.sh            # 也可以直接跑
# ============================================================================

set -u

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[0;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

PASS=0; WARN=0; FAIL=0

ok()   { echo -e "  ${GREEN}[OK]${RESET}   $*"; PASS=$((PASS+1)); }
warn() { echo -e "  ${YELLOW}[警告]${RESET} $*"; WARN=$((WARN+1)); }
bad()  { echo -e "  ${RED}[失败]${RESET} $*"; FAIL=$((FAIL+1)); }
step() { echo -e "\n${CYAN}${BOLD}── $* ${RESET}"; }

echo -e "${BOLD}"
echo "════════════════════════════════════════════════════════════════"
echo "  硬件层检查（不需要 ROS）"
echo "  $(date '+%Y-%m-%d %H:%M:%S')  主机：$(hostname)"
echo "════════════════════════════════════════════════════════════════"
echo -e "${RESET}"

# ============================================================================
step "1. 关键设备节点"
# ============================================================================

# 设备名 -> 用途 -> 是否必需
# 【必需】= 1 表示导航离了它不行
check_dev() {
    local dev="$1" purpose="$2" required="${3:-1}" hint="${4:-}"
    if [ -e "$dev" ]; then
        local perm
        perm=$(stat -c '%a %U:%G' "$dev" 2>/dev/null || echo '?')
        if [ -r "$dev" ] && [ -w "$dev" ]; then
            ok "$(printf '%-26s' "$dev") $purpose  [$perm]"
        else
            warn "$(printf '%-26s' "$dev") 权限不足 [$perm]"
            [ -n "$hint" ] && echo "             → $hint"
        fi
    else
        if [ "$required" = "1" ]; then
            bad "$(printf '%-26s' "$dev") $purpose —— 不存在"
        else
            warn "$(printf '%-26s' "$dev") $purpose —— 不存在（可选）"
        fi
        [ -n "$hint" ] && echo "             → $hint"
    fi
}

echo "  --- 底盘 ---"
check_dev /dev/stm32            "STM32 底盘（实验室实车）" 1 \
    "检查 USB 线；udev 规则：base_control 包；或改用 /dev/ttyACM0"
check_dev /dev/ttyACM0          "Arduino 底盘（备选配置）" 0 \
    "仅在 base_driver:=arduino 时需要"

echo "  --- 激光雷达 ---"
check_dev /dev/carebot_laser_ls "镭神 M10（实验室实车）" 1 \
    "检查 USB 口是否插对；udev 规则：lslidar_driver/scripts/"
check_dev /dev/laser            "ls01d 系列（备选）" 0 \
    "仅在 laser_driver:=ls01d 时需要"
check_dev /dev/rplidar          "RPLidar（备选）" 0 ""

echo "  --- IMU ---"
check_dev /dev/fdilink_ahrs     "fdilink AHRS（实验室实车）" 0 \
    "缺 IMU 仍可导航（纯轮式里程计），但定位稳定性下降"
check_dev /dev/art_imu          "art_imu_01（备选）" 0 ""

echo "  --- 其他 ---"
check_dev /dev/IOT_net          "物联网串口网关" 0 "与导航无关，属 carebot_iot 范畴"

# ============================================================================
step "2. USB 设备（按 VID 识别）"
# ============================================================================

if command -v lsusb >/dev/null 2>&1; then
    # 常见的 USB 转串口芯片
    for spec in \
        "1a86:7523|CH340/CH341 串口芯片（STM32 或 IMU 常用）" \
        "10c4:ea60|CP210x 串口芯片（雷达常用）" \
        "0403:6001|FTDI 串口芯片" \
        "2bc5:|Orbbec 深度相机" \
        "10d6:|讯飞麦克风阵列" ; do
        vid="${spec%%|*}"; desc="${spec##*|}"
        if lsusb 2>/dev/null | grep -qi "$vid"; then
            n=$(lsusb 2>/dev/null | grep -ci "$vid")
            ok "$desc  找到 $n 个（$vid）"
        else
            if [ "$vid" = "2bc5:" ] || [ "$vid" = "10d6:" ]; then
                warn "$desc  未找到（$vid）"
            else
                warn "$desc  未找到（$vid）"
            fi
        fi
    done
else
    warn "没有 lsusb 命令，跳过 USB 检查（安装：sudo apt install usbutils）"
fi

# ============================================================================
step "3. 串口占用冲突"
# ============================================================================

# 同一个串口被两个进程打开会导致数据错乱 —— 是很隐蔽的故障源
if command -v lsof >/dev/null 2>&1; then
    FOUND=0
    for dev in /dev/stm32 /dev/carebot_laser_ls /dev/laser /dev/fdilink_ahrs; do
        [ -e "$dev" ] || continue
        users=$(sudo lsof "$dev" 2>/dev/null | tail -n +2 | awk '{print $1" (pid "$2")"}' | sort -u | tr '\n' ' ')
        if [ -n "$users" ]; then
            warn "$(printf '%-26s' "$dev") 被占用：$users"
            warn "             → 若同一设备被两个进程打开，数据会错乱。先杀掉旧进程"
            FOUND=1
        fi
    done
    [ "$FOUND" = "0" ] && ok "没有串口被占用"
else
    warn "没有 lsof 命令，跳过占用检查（安装：sudo apt install lsof）"
fi

# ============================================================================
step "4. udev 规则"
# ============================================================================

RULE_COUNT=$(ls /etc/udev/rules.d/*.rules 2>/dev/null | wc -l)
if [ "$RULE_COUNT" -gt 0 ]; then
    ok "已安装 $RULE_COUNT 个 udev 规则文件"
    echo "       涉及设备别名的规则："
    grep -l 'SYMLINK' /etc/udev/rules.d/*.rules 2>/dev/null | while read -r f; do
        syms=$(grep -o 'SYMLINK+="[^"]*"' "$f" 2>/dev/null | sed 's/SYMLINK+="//;s/"//' | tr '\n' ' ')
        [ -n "$syms" ] && echo "         $(basename "$f") → $syms"
    done
    echo "       ⚠️ 注意：art_imu 与 IOT_net 的 VID:PID 可能都是 1a86:7523，"
    echo "          靠 USB 物理端口号区分。插错口会让规则互相覆盖。"
else
    warn "没有找到 udev 规则 —— 设备名可能不稳定"
    echo "       从组委会源码安装（在 rosCode/src 下执行）："
    echo "         sudo cp art_imu_01/config/art_imu.rules /etc/udev/rules.d/ 2>/dev/null"
    echo "         sudo cp lslidar_driver/scripts/*.rules   /etc/udev/rules.d/ 2>/dev/null"
    echo "         sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

# ============================================================================
step "5. 当前登录用户与权限"
# ============================================================================

CUR_USER="$(whoami)"
ok "当前用户：$CUR_USER"
if id -nG "$CUR_USER" 2>/dev/null | grep -qw dialout; then
    ok "已在 dialout 组（可访问串口）"
else
    warn "不在 dialout 组 —— 串口可能无权限访问"
    echo "       → sudo usermod -aG dialout $CUR_USER  然后重新登录"
fi

# ============================================================================
step "6. ROS 环境"
# ============================================================================

if [ -n "${ROS_DISTRO:-}" ]; then
    ok "ROS_DISTRO=$ROS_DISTRO"
else
    warn "ROS_DISTRO 未设置 —— 当前 shell 没 source ROS 环境"
    echo "       → source /opt/ros/kinetic/setup.bash"
fi

if [ -f "$HOME/omniWheelCareRobot/rosCode/devel/setup.bash" ]; then
    ok "找到工作空间 devel/setup.bash"
else
    warn "未找到 ~/omniWheelCareRobot/rosCode/devel/setup.bash"
    echo "       → 工作空间可能未编译，或路径不同"
fi

# ============================================================================
step "7. 系统资源"
# ============================================================================

echo "  --- 内存 ---"
free -h 2>/dev/null | sed 's/^/    /' | head -2

echo "  --- CPU 负载 ---"
uptime | sed 's/^/    /'

echo "  --- 磁盘 ---"
df -h / 2>/dev/null | tail -1 | sed 's/^/    /'

AVAIL_KB=$(df --output=avail / 2>/dev/null | tail -1 | tr -d ' ')
if [ -n "$AVAIL_KB" ] && [ "$AVAIL_KB" -lt 1048576 ]; then
    warn "根分区可用空间不足 1GB —— 建图/录包可能失败"
fi

# ============================================================================
#  汇总
# ============================================================================
echo
echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"
if [ "$FAIL" -gt 0 ]; then
    echo -e "${RED}${BOLD}  失败 $FAIL 项，警告 $WARN 项，通过 $PASS 项${RESET}"
    echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"
    echo
    echo -e "  ${RED}存在缺失的必需设备，ROS 启动后会报错。${RESET}"
    echo "  先解决上面标 [失败] 的项，再启动 ROS。"
    exit 1
elif [ "$WARN" -gt 0 ]; then
    echo -e "${YELLOW}${BOLD}  通过 $PASS 项，警告 $WARN 项${RESET}"
    echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"
    echo
    echo "  必需设备齐全，可以启动 ROS。但建议先看看上面的警告项。"
    echo
    echo "  下一步："
    echo "    roslaunch carebot_nav_kit bringup.launch"
    echo "    rosrun carebot_nav_kit nav_check.py"
    exit 0
else
    echo -e "${GREEN}${BOLD}  全部通过（$PASS 项）${RESET}"
    echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"
    echo
    echo "  硬件层没问题，可以启动 ROS 做软件层检查："
    echo "    roslaunch carebot_nav_kit mapping.launch"
    exit 0
fi
