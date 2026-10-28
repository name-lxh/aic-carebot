#!/bin/bash
# ============================================================================
#  run_recording.sh — 智慧养老护理赛题：线上赛双机位录制辅助
# ============================================================================
#  赛题线上赛要求：
#    · 一镜到底，不可分段拼接、剪辑、加速
#    · 双机位：1 号机位显示全局地图与设备运行；2 号机位跟随设备显示上位机实时状态
#    · MP4 格式，命名为「1号机位视频」「2号机位视频」
#    · 技术文档命名为「参赛院校-团队名称-技术报告」
#
#  本脚本负责把"2 号机位的上位机状态画面"准备好，并可选直接调 ffmpeg 录制。
#
#  用法：
#      ./run_recording.sh                       # 只启动状态看板（推荐先看效果）
#      ./run_recording.sh --record              # 启动看板 + 开始录制
#      ./run_recording.sh --record --out ~/videos
#      ./run_recording.sh --single-window       # 只开一个大图窗（画面更干净）
#
#  画面布局（2 号机位建议）：
#      ┌──────────────────────┬──────────────────────┐
#      │  /qr/debug_image     │  /face/debug_image   │
#      │  二维码检测框        │  人脸识别标注        │
#      ├──────────────────────┴──────────────────────┤
#      │  终端窗口：                                  │
#      │    rostopic echo /eldercare/status           │
#      │    rostopic echo /iot/state                  │
#      └─────────────────────────────────────────────┘
# ============================================================================

set -u

RECORD=0
SINGLE_WINDOW=0
OUT_DIR="${HOME}/eldercare_videos"
FPS=15
SIZE="1280x720"

while [ $# -gt 0 ]; do
    case "$1" in
        --record)         RECORD=1 ;;
        --single-window)  SINGLE_WINDOW=1 ;;
        --out)            OUT_DIR="$2"; shift ;;
        --fps)            FPS="$2"; shift ;;
        --size)           SIZE="$2"; shift ;;
        -h|--help)        sed -n '2,32p' "$0"; exit 0 ;;
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
echo "   智慧养老护理赛题 — 双机位录制辅助"
echo "   $(date '+%Y-%m-%d %H:%M:%S')"
echo "════════════════════════════════════════════════════════════════════"
echo -e "${RESET}"

# ---------------------------------------------------------------- 环境
step "检查环境"
# shellcheck disable=SC1091
source /opt/ros/kinetic/setup.bash 2>/dev/null || { fail "ROS 环境不可用"; exit 1; }
# shellcheck disable=SC1091
source "${HOME}/omniWheelCareRobot/rosCode/devel/setup.bash" 2>/dev/null || true
ok "ROS 环境就绪"

if ! command -v rqt_image_view >/dev/null 2>&1; then
    fail "未找到 rqt_image_view，请安装：sudo apt-get install ros-kinetic-rqt-image-view"
    exit 1
fi
ok "rqt_image_view 可用"

HAVE_FFMPEG=0
if command -v ffmpeg >/dev/null 2>&1; then
    HAVE_FFMPEG=1
    ok "ffmpeg 可用"
else
    warn "未找到 ffmpeg（--record 将不可用）"
    warn "安装：sudo apt-get install ffmpeg"
fi

# 确认有画面源
step "检查画面话题"
for topic in /qr/debug_image /face/debug_image; do
    if rostopic list 2>/dev/null | grep -qx "$topic"; then
        ok "$topic 存在"
    else
        warn "$topic 不存在 —— 对应节点可能未启动"
    fi
done

# ---------------------------------------------------------------- 看板
step "启动状态看板"

mkdir -p /tmp/carebot_recording

# 文本状态面板：把关键话题落到文件，供另一个终端 tail -f 显示
cat > /tmp/carebot_recording/status_panel.sh <<'PANEL'
#!/bin/bash
# 2 号机位的文本状态面板（另开一个终端运行本脚本）
echo "════════════════════════════════════════════════════════════"
echo "  智慧养老护理 — 上位机实时状态"
echo "════════════════════════════════════════════════════════════"
echo
echo "[任务状态]"
rostopic echo /eldercare/mission_state &
P1=$!
echo
echo "[状态详情]"
rostopic echo /eldercare/status &
P2=$!
echo
echo "[物联网设备状态]"
rostopic echo /iot/state &
P3=$!
echo
echo "[语音播报]"
rostopic echo /voice/status &
P4=$!
trap "kill $P1 $P2 $P3 $P4 2>/dev/null" EXIT
wait
PANEL
chmod +x /tmp/carebot_recording/status_panel.sh
ok "文本状态面板：/tmp/carebot_recording/status_panel.sh"

# 图像窗口
if [ "$SINGLE_WINDOW" = "1" ]; then
    rqt_image_view /qr/debug_image >/dev/null 2>&1 &
    ok "已打开单个图像窗口（/qr/debug_image）"
    warn "想看人脸画面就改成：rqt_image_view /face/debug_image"
else
    rqt_image_view /qr/debug_image >/dev/null 2>&1 &
    sleep 2
    rqt_image_view /face/debug_image >/dev/null 2>&1 &
    ok "已打开两个图像窗口（二维码 + 人脸）"
fi
sleep 2

# ---------------------------------------------------------------- 录制
if [ "$RECORD" = "1" ]; then
    if [ "$HAVE_FFMPEG" = "0" ]; then
        fail "--record 需要 ffmpeg，请先安装"
        exit 1
    fi

    step "开始录制 2 号机位"
    mkdir -p "$OUT_DIR"
    STAMP="$(date +%Y%m%d_%H%M%S)"
    OUT_FILE="${OUT_DIR}/2号机位视频_${STAMP}.mp4"

    warn "比赛规则：一镜到底，不可剪辑、不可加速。"
    warn "录制期间不要停止本脚本，也不要切换画面布局。"
    echo
    echo "  输出文件：$OUT_FILE"
    echo "  录制规格：${SIZE} @ ${FPS}fps"
    echo "  抓屏设备：x11grab（需要 DISPLAY 可用）"
    echo
    echo -e "  ${BOLD}按 Ctrl+C 停止录制${RESET}"
    echo

    # shellcheck disable=SC2086
    ffmpeg -f x11grab -framerate "$FPS" -video_size "$SIZE" -i "${DISPLAY:-:0}" \
           -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p \
           "$OUT_FILE"

    echo
    ok "录制结束：$OUT_FILE"
    if [ -f "$OUT_FILE" ]; then
        echo "  文件大小：$(du -h "$OUT_FILE" | cut -f1)"
        echo
        echo -e "  ${BOLD}请立即重命名为组委会要求的名称：${RESET}"
        echo "    2号机位视频.mp4"
    fi
else
    echo
    echo -e "${BOLD}════════════════════════════════════════════════════════════════════${RESET}"
    echo -e "${GREEN}${BOLD}   看板已就绪（未录制）${RESET}"
    echo -e "${BOLD}════════════════════════════════════════════════════════════════════${RESET}"
    echo
    echo -e "  ${BOLD}另开一个终端运行文本状态面板：${RESET}"
    echo -e "    ${CYAN}/tmp/carebot_recording/status_panel.sh${RESET}"
    echo
    echo -e "  ${BOLD}确认画面满意后，重新运行本脚本并加 --record：${RESET}"
    echo -e "    ${CYAN}$0 --record${RESET}"
fi

# ---------------------------------------------------------------- 清单
echo
step "线上赛提交清单"
cat <<'CHECKLIST'
  [ ] 1 号机位视频：显示全局地图与设备运行情况
        · 整场地俯视，能看到机器人从起点出发、按顺序走完三个房间、回到起点
        · 建议用手机/相机固定机位，画面里能看到整个 3m×3m 场地
  [ ] 2 号机位视频：跟随设备显示上位机实时状态
        · 本脚本开启的 rqt_image_view（二维码/人脸调试图）
        · 加上 status_panel.sh 的文本状态
  [ ] 两个视频均为一镜到底，无剪辑、无加速、无分段
  [ ] 命名：1号机位视频.mp4 / 2号机位视频.mp4
  [ ] 技术报告命名：参赛院校-团队名称-技术报告
  [ ] 音画清晰稳定
CHECKLIST
