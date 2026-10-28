#!/bin/bash
# ============================================================================
#  run_competition.sh — 智慧养老护理赛题：比赛当日一键启动
# ============================================================================
#  作用：环境检查 → 启动全部节点 → 等待就绪 → 等裁判示意 → 触发任务
#
#  用法：
#      ./run_competition.sh                      # 交互确认模式（推荐）
#      ./run_competition.sh --auto               # 不做交互确认，启动后等 START 文件
#      ./run_competition.sh --no-nav             # 不启动导航（导航已在另一个终端跑）
#      ./run_competition.sh --tts-bin /path/xf_tts_cli
#
#  说明：
#    * 本脚本**不启动导航栈**时也需要 roscore 在跑；默认会连导航一起启动。
#    * 触发任务有两种方式（二选一）：
#        1) 另一个终端执行：rostopic pub -1 /eldercare/start std_msgs/Bool true
#        2) 本脚本轮询 /tmp/carebot_go 文件：touch /tmp/carebot_go
#      推荐方式 1，更直观。
# ============================================================================

set -u

# ---------------------------------------------------------------- 参数解析
START_NAV=1
AUTO=0
TTS_BIN=""
MAP_FILE=""
HA_URL="http://192.168.8.190:8123"
MASTER_IDS="3"
DRY_RUN="false"

while [ $# -gt 0 ]; do
    case "$1" in
        --auto)       AUTO=1 ;;
        --no-nav)     START_NAV=0 ;;
        --dry-run)    DRY_RUN="true" ;;
        --tts-bin)    TTS_BIN="$2"; shift ;;
        --map)        MAP_FILE="$2"; shift ;;
        --ha-url)     HA_URL="$2"; shift ;;
        --master-ids) MASTER_IDS="$2"; shift ;;
        -h|--help)
            sed -n '2,25p' "$0"
            exit 0 ;;
        *)
            echo "未知参数：$1（用 --help 查看用法）" >&2
            exit 2 ;;
    esac
    shift
done

RED='\033[31;1m'; GREEN='\033[32;1m'; YELLOW='\033[33;1m'
CYAN='\033[36;1m'; BOLD='\033[1m'; RESET='\033[0m'

step()  { echo -e "\n${CYAN}${BOLD}▶ $*${RESET}"; }
ok()    { echo -e "  ${GREEN}✔${RESET} $*"; }
warn()  { echo -e "  ${YELLOW}⚠${RESET} $*"; }
fail()  { echo -e "  ${RED}✘${RESET} $*"; }

echo -e "${BOLD}"
echo "════════════════════════════════════════════════════════════════════"
echo "   智慧养老护理赛题 — 比赛启动"
echo "   $(date '+%Y-%m-%d %H:%M:%S')"
echo "════════════════════════════════════════════════════════════════════"
echo -e "${RESET}"

# ---------------------------------------------------------------- 1. 环境
step "1/5 加载 ROS 环境"

WORKSPACE="${HOME}/omniWheelCareRobot/rosCode"
if [ -f /opt/ros/kinetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/kinetic/setup.bash
    ok "已 source /opt/ros/kinetic/setup.bash"
else
    fail "找不到 /opt/ros/kinetic/setup.bash —— ROS 未安装或版本不符"
    exit 1
fi

if [ -f "${WORKSPACE}/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${WORKSPACE}/devel/setup.bash"
    ok "已 source ${WORKSPACE}/devel/setup.bash"
else
    fail "找不到 ${WORKSPACE}/devel/setup.bash —— 请先 catkin_make"
    exit 1
fi

# ---------------------------------------------------------------- 2. 设备
step "2/5 检查关键设备节点"

DEV_MISSING=0
check_dev() {
    local path="$1" label="$2" optional="${3:-0}"
    if [ -e "$path" ]; then
        ok "$label ($path)"
    elif [ "$optional" = "1" ]; then
        warn "$label ($path) 不存在 —— 可选，忽略"
    else
        fail "$label ($path) 不存在"
        DEV_MISSING=1
    fi
}

check_dev /dev/stm32            "底盘 STM32"
check_dev /dev/carebot_laser_ls "镭神 M10 雷达"
check_dev /dev/fdilink_ahrs     "IMU"
check_dev /dev/IOT_net          "物联网串口网关" 1

if command -v lsusb >/dev/null 2>&1; then
    lsusb | grep -qi '2bc5' && ok "Astra 深度相机已识别" || fail "未找到 Astra 相机 (VID 2bc5)"
    lsusb | grep -qi '10d6' && ok "讯飞麦克风阵列已识别" || fail "未找到麦克风阵列 (VID 10d6)"
fi

if [ "$DEV_MISSING" = "1" ]; then
    echo
    fail "关键设备缺失，继续启动大概率失败。"
    if [ "$AUTO" = "0" ]; then
        read -r -p "  仍要继续？(y/N) " ans
        case "$ans" in y|Y) ;; *) exit 1 ;; esac
    else
        exit 1
    fi
fi

# ---------------------------------------------------------------- 3. 令牌
step "3/5 检查 HomeAssistant 令牌"

if [ -n "${HA_TOKEN:-}" ]; then
    ok "HA_TOKEN 已从环境变量设置"
elif [ -f /etc/carebot/ha_token ]; then
    ok "找到令牌文件 /etc/carebot/ha_token"
    export HA_TOKEN
    HA_TOKEN="$(cat /etc/carebot/ha_token)"
else
    warn "未找到 HA_TOKEN 环境变量，也没有 /etc/carebot/ha_token"
    warn "物联网相关任务（灯/窗帘/风扇/传感器，共 30 分）将无法工作"
    if [ "$AUTO" = "0" ]; then
        read -r -p "  仍要继续？(y/N) " ans
        case "$ans" in y|Y) ;; *) exit 1 ;; esac
    fi
fi

# ---------------------------------------------------------------- 4. 启动
step "4/5 启动系统"

if [ -z "$MAP_FILE" ]; then
    MAP_FILE="$(rospack find carebot_nav_kit)/maps/mymap.yaml"
fi
if [ ! -f "$MAP_FILE" ]; then
    fail "地图文件不存在：$MAP_FILE"
    warn "请先建图并保存到 carebot_nav_kit/maps/，或用 --map 指定"
    exit 1
fi
ok "地图：$MAP_FILE"

LAUNCH_ARGS=" map_file:=$MAP_FILE ha_base_url:=$HA_URL master_ids:=$MASTER_IDS dry_run:=$DRY_RUN"
if [ "$START_NAV" = "0" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS start_nav:=false"
fi
if [ -n "$TTS_BIN" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS tts_bin:=$TTS_BIN"
    ok "TTS 合成器：$TTS_BIN"
else
    warn "未指定 --tts-bin，语音播报会退化到 espeak（中文效果差）"
    warn "建议先编译：carebot_voice/scripts/build_xf_tts_cli.sh"
fi

LOG_FILE="/tmp/carebot_competition_$(date +%Y%m%d_%H%M%S).log"
echo "  启动日志：$LOG_FILE"
echo "  启动命令：roslaunch carebot_eldercare eldercare_full.launch $LAUNCH_ARGS"
echo

# shellcheck disable=SC2086
roslaunch carebot_eldercare eldercare_full.launch $LAUNCH_ARGS 2>&1 | tee "$LOG_FILE" &
LAUNCH_PID=$!

echo "  roslaunch PID = $LAUNCH_PID"

# 退出时清理，避免留下孤儿 roslaunch 进程
cleanup() {
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo -e "\n${YELLOW}  正在停止 roslaunch (PID $LAUNCH_PID)...${RESET}"
        kill "$LAUNCH_PID" 2>/dev/null || true
        sleep 2
        kill -9 "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "  等待各节点就绪（30 秒）..."
sleep 30

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    fail "roslaunch 已退出，请查看日志：$LOG_FILE"
    exit 1
fi
ok "roslaunch 仍在运行"

# ---------------------------------------------------------------- 5. 自检
step "5/5 运行自检"

if rosrun carebot_eldercare preflight_check.py 2>&1 | tee /tmp/carebot_preflight.log | tail -20; then
    :
fi

if grep -q '失败 0 项' /tmp/carebot_preflight.log 2>/dev/null; then
    ok "自检通过"
else
    warn "自检存在失败项，请查看 /tmp/carebot_preflight.log"
    warn "（若是已知的可选项，可以继续）"
fi

# ---------------------------------------------------------------- 就绪
echo
echo -e "${BOLD}════════════════════════════════════════════════════════════════════${RESET}"
echo -e "${GREEN}${BOLD}   系统已就绪${RESET}"
echo -e "${BOLD}════════════════════════════════════════════════════════════════════${RESET}"
echo
echo -e "  ${BOLD}接下来：${RESET}"
echo -e "    1. 把机器人放到出发区"
echo -e "    2. 在 RViz 里用 ${CYAN}2D Pose Estimate${RESET} 给机器人一个准确初始位姿"
echo -e "    3. 向裁判示意，然后触发任务："
echo
echo -e "       ${CYAN}rostopic pub -1 /eldercare/start std_msgs/Bool true${RESET}"
echo
echo -e "    或在本脚本所在终端执行：  ${CYAN}touch /tmp/carebot_go${RESET}"
echo
echo -e "  ${BOLD}监控：${RESET}"
echo -e "    rostopic echo /eldercare/mission_state     # 当前状态"
echo -e "    rostopic echo /eldercare/status            # 人类可读状态"
echo -e "    rqt_image_view /qr/debug_image             # 二维码调试图"
echo -e "    rqt_image_view /face/debug_image           # 人脸调试图"
echo
echo -e "  ${BOLD}中止（紧急）：${RESET}"
echo -e "    ${CYAN}rostopic pub -1 /eldercare/abort std_msgs/Bool true${RESET}"
echo
echo -e "  退出本脚本不会停止 roslaunch。停止全部：${CYAN}kill $LAUNCH_PID${RESET}"
echo

# ---------------------------------------------------------------- 等待触发
rm -f /tmp/carebot_go

if [ "$AUTO" = "0" ]; then
    echo -e "${BOLD}  按回车键开始任务（或 Ctrl+C 退出脚本，节点继续运行）${RESET}"
    if read -r _; then
        rostopic pub -1 /eldercare/start std_msgs/Bool true
        ok "任务已触发"
    fi
else
    echo -e "${BOLD}  --auto 模式：等待 /tmp/carebot_go 或 /eldercare/start${RESET}"
    echo "  （也可以直接 rostopic pub 触发，本脚本只在文件出现时补一次触发）"
    while kill -0 "$LAUNCH_PID" 2>/dev/null; do
        if [ -f /tmp/carebot_go ]; then
            rm -f /tmp/carebot_go
            rostopic pub -1 /eldercare/start std_msgs/Bool true
            ok "任务已触发（通过 /tmp/carebot_go）"
        fi
        sleep 1
    done
fi

wait "$LAUNCH_PID"
