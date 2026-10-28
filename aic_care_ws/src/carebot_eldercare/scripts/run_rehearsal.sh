#!/bin/bash
# ============================================================================
#  run_rehearsal.sh — 智慧养老护理赛题：整轮彩排
# ============================================================================
#  作用：启动系统 → 等就绪 → 自动连跑 N 轮任务 → 汇总成功率与用时 → 生成报告数据
#
#  用法：
#      ./run_rehearsal.sh                    # 默认跑 5 轮
#      ./run_rehearsal.sh -n 10              # 跑 10 轮
#      ./run_rehearsal.sh -n 3 --simple      # 只测逻辑，不控实物、不测导航
#      ./run_rehearsal.sh -n 5 --tts-bin /path/xf_tts_cli
#
#  与 run_competition.sh 的区别：
#      run_competition.sh  比赛用，交互确认、等裁判示意、单轮
#      run_rehearsal.sh    赛前用，自动连跑多轮、统计成功率
# ============================================================================

set -u

RUNS=5
SIMPLE=0
TTS_BIN=""
MAP_FILE=""
HA_URL="http://192.168.8.190:8123"
RUNS_DIR="${HOME}/eldercare_runs"
TIMEOUT=300

while [ $# -gt 0 ]; do
    case "$1" in
        -n|--runs)     RUNS="$2"; shift ;;
        --simple)      SIMPLE=1 ;;
        --tts-bin)     TTS_BIN="$2"; shift ;;
        --map)         MAP_FILE="$2"; shift ;;
        --ha-url)      HA_URL="$2"; shift ;;
        --runs-dir)    RUNS_DIR="$2"; shift ;;
        --timeout)     TIMEOUT="$2"; shift ;;
        -h|--help)     sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

RED='\033[31;1m'; GREEN='\033[32;1m'; YELLOW='\033[33;1m'
CYAN='\033[36;1m'; BOLD='\033[1m'; RESET='\033[0m'
step() { echo -e "\n${CYAN}${BOLD}▶ $*${RESET}"; }
ok()   { echo -e "  ${GREEN}✔${RESET} $*"; }
warn() { echo -e "  ${YELLOW}⚠${RESET} $*"; }
fail() { echo -e "  ${RED}✘${RESET} $*"; }

echo -e "${BOLD}"
echo "════════════════════════════════════════════════════════════════════"
echo "   智慧养老护理赛题 — 整轮彩排（$RUNS 轮）"
echo "   $(date '+%Y-%m-%d %H:%M:%S')"
echo "════════════════════════════════════════════════════════════════════"
echo -e "${RESET}"

# ---------------------------------------------------------------- 环境
step "加载 ROS 环境"
WORKSPACE="${HOME}/omniWheelCareRobot/rosCode"
# shellcheck disable=SC1091
source /opt/ros/kinetic/setup.bash 2>/dev/null || { fail "ROS 环境不可用"; exit 1; }
# shellcheck disable=SC1091
source "${WORKSPACE}/devel/setup.bash" 2>/dev/null || { fail "工作空间未编译"; exit 1; }
ok "ROS 环境就绪"

if [ -f /etc/carebot/ha_token ] && [ -z "${HA_TOKEN:-}" ]; then
    HA_TOKEN="$(cat /etc/carebot/ha_token)"
    export HA_TOKEN
fi

# ---------------------------------------------------------------- 启动
step "启动系统"

LAUNCH_ARGS=" ha_base_url:=$HA_URL"
if [ -n "$MAP_FILE" ]; then LAUNCH_ARGS="$LAUNCH_ARGS map_file:=$MAP_FILE"; fi
if [ -n "$TTS_BIN" ]; then LAUNCH_ARGS="$LAUNCH_ARGS tts_bin:=$TTS_BIN"; fi

if [ "$SIMPLE" = "1" ]; then
    warn "简化模式：不启动导航与相机，状态机 dry_run（只验证逻辑与感知链路）"
    LAUNCH_ARGS="$LAUNCH_ARGS start_nav:=false start_camera:=false dry_run:=true"
else
    ok "完整模式：导航 + 相机 + 语音 + 物联网 + 感知 + 编排"
fi

LOG_FILE="/tmp/carebot_rehearsal_$(date +%Y%m%d_%H%M%S).log"
echo "  启动日志：$LOG_FILE"

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

echo "  等待节点就绪（35 秒）..."
sleep 35

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    fail "roslaunch 已退出，日志：$LOG_FILE"
    exit 1
fi
ok "系统在运行"

# ---------------------------------------------------------------- 自检
step "系统自检"
rosrun carebot_eldercare preflight_check.py > /tmp/carebot_preflight.log 2>&1 || true
tail -30 /tmp/carebot_preflight.log
if grep -q '失败 0 项' /tmp/carebot_preflight.log 2>/dev/null; then
    ok "自检通过"
else
    warn "自检有失败项，彩排可能不顺利（详见 /tmp/carebot_preflight.log）"
fi

# ---------------------------------------------------------------- 彩排
step "开始 $RUNS 轮彩排"
echo "  日志目录：$RUNS_DIR"
echo "  单轮超时：${TIMEOUT} 秒"
echo

REHEARSAL_ARGS=(--mode repeat --runs "$RUNS" --runs-dir "$RUNS_DIR" --timeout "$TIMEOUT" --force)
if [ "$SIMPLE" = "1" ]; then REHEARSAL_ARGS+=(--dry-run); fi

rosrun carebot_eldercare rehearsal.py "${REHEARSAL_ARGS[@]}"
REHEARSAL_RC=$?

# ---------------------------------------------------------------- 收尾
echo
if [ "$REHEARSAL_RC" = "0" ]; then
    ok "全部 $RUNS 轮成功"
else
    warn "存在失败轮次，请查看 $RUNS_DIR 下的日志与汇总"
fi

echo
echo -e "${BOLD}  产出：${RESET}"
echo "    事件日志    : $RUNS_DIR/run*.jsonl"
echo "    实测数据汇总: $RUNS_DIR/实测数据汇总.md   ← 可直接粘进技术报告"
echo "    运行清单    : $RUNS_DIR/rehearsal_manifest.json"
echo "    启动日志    : $LOG_FILE"
echo
echo -e "  ${BOLD}停止系统：${RESET}kill $LAUNCH_PID"
echo

if [ "$REHEARSAL_RC" != "0" ]; then
    exit "$REHEARSAL_RC"
fi
