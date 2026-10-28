#!/bin/bash
# ============================================================================
#  save_map.sh — 安全地保存 SLAM 地图
# ============================================================================
#  为什么需要脚本而不是直接 map_saver：
#    1. 地图要存到**两个地方**：本包 maps/ 目录（供导航用）和时间戳备份
#       （防止覆盖上次的好地图）
#    2. 保存前要检查 /map 话题是否有数据 —— 没有则 map_saver 会生成一张空图
#    3. 保存后要校验生成的文件是否有效（.pgm 大小、.yaml 内容）
#
#  用法：
#      rosrun carebot_nav_kit save_map.sh                    # 交互式
#      rosrun carebot_nav_kit save_map.sh -n mymap           # 指定名字
#      rosrun carebot_nav_kit save_map.sh -n mymap -y        # 跳过确认
# ============================================================================

set -u

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[0;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
ok()   { echo -e "  ${GREEN}✔${RESET} $*"; }
warn() { echo -e "  ${YELLOW}⚠${RESET} $*"; }
err()  { echo -e "  ${RED}✘${RESET} $*"; }
step() { echo -e "\n${CYAN}▶ $*${RESET}"; }

MAP_NAME="mymap"
SKIP_CONFIRM=0
PKG_DIR=""

# --- 参数 ---------------------------------------------------------------
while [ $# -gt 0 ]; do
    case "$1" in
        -n|--name)    MAP_NAME="$2"; shift ;;
        -y|--yes)     SKIP_CONFIRM=1 ;;
        -h|--help)    sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

echo -e "${BOLD}"
echo "════════════════════════════════════════════════════════════════"
echo "  保存地图"
echo "════════════════════════════════════════════════════════════════"
echo -e "${RESET}"

# --- 环境 ---------------------------------------------------------------
if ! command -v rosrun >/dev/null 2>&1; then
    err "找不到 rosrun —— 请先 source ROS 环境"
    exit 1
fi

PKG_DIR="$(rospack find carebot_nav_kit 2>/dev/null || true)"
if [ -z "$PKG_DIR" ]; then
    err "找不到 carebot_nav_kit 包 —— 请先 source 工作空间"
    echo "    source ~/omniWheelCareRobot/rosCode/devel/setup.bash"
    exit 1
fi
ok "包目录：$PKG_DIR"

MAPS_DIR="$PKG_DIR/maps"
mkdir -p "$MAPS_DIR"

# --- 检查 /map 是否有数据 -----------------------------------------------
step "检查 /map 话题"

if ! rostopic list 2>/dev/null | grep -qx "/map"; then
    err "/map 话题不存在 —— SLAM 没在运行？"
    echo "    请先启动：roslaunch carebot_nav_kit mapping.launch"
    exit 1
fi

# 读一帧地图信息，确认尺寸合理
MAP_INFO=$(timeout 10 rostopic echo -n 1 /map/info 2>/dev/null || echo "")
if [ -z "$MAP_INFO" ]; then
    warn "读不到 /map/info，仍然尝试保存"
else
    W=$(echo "$MAP_INFO" | grep -A2 'width' | grep 'data:' | head -1 | awk '{print $2}')
    echo "  地图宽度像素：${W:-未知}"
fi

# 等待收到实际地图数据
echo "  等待地图数据…"
GOT=0
for i in $(seq 1 15); do
    if timeout 5 rostopic echo -n 1 /map 2>/dev/null | head -3 | grep -q 'header'; then
        GOT=1
        break
    fi
    sleep 1
done

if [ "$GOT" = "0" ]; then
    err "15 秒内收不到地图数据"
    echo "    可能原因：gmapping 未启动 / 机器人还没动过（地图为空）"
    echo "    建议：先遥控机器人走一小段，让地图有内容再保存"
    exit 1
fi
ok "地图数据正常"

# --- 确认 ---------------------------------------------------------------
if [ "$SKIP_CONFIRM" = "0" ]; then
    echo
    echo -e "  ${BOLD}即将保存为：${RESET}$MAP_NAME"
    echo -e "  保存到    ：$MAPS_DIR/"
    echo -e "  同时备份到：$MAPS_DIR/history/${MAP_NAME}_<时间戳>"
    echo
    echo -e "  ${YELLOW}提示：保存前请确认地图已建完整（三个房间都走过、闭环了）${RESET}"
    echo
    printf "  确认保存？(y/N) "
    read -r ans
    case "$ans" in
        y|Y) ;;
        *) echo "  已取消"; exit 0 ;;
    esac
fi

# --- 保存 ---------------------------------------------------------------
step "保存地图"

TMPDIR_MAP="$(mktemp -d)"
if ! rosrun map_server map_saver -f "$TMPDIR_MAP/$MAP_NAME" 2>&1 | tail -5; then
    err "map_saver 执行失败"
    rm -rf "$TMPDIR_MAP"
    exit 1
fi

PGM="$TMPDIR_MAP/$MAP_NAME.pgm"
YAML="$TMPDIR_MAP/$MAP_NAME.yaml"

if [ ! -f "$PGM" ] || [ ! -f "$YAML" ]; then
    err "map_saver 未生成预期文件"
    ls -la "$TMPDIR_MAP"
    rm -rf "$TMPDIR_MAP"
    exit 1
fi

PGM_SIZE=$(stat -c%s "$PGM" 2>/dev/null || echo 0)
if [ "$PGM_SIZE" -lt 1000 ]; then
    err "生成的 .pgm 只有 $PGM_SIZE 字节，地图基本是空的"
    echo "    原因：SLAM 可能还没建出内容，或 /map 是空图"
    rm -rf "$TMPDIR_MAP"
    exit 1
fi
ok "生成成功（.pgm $PGM_SIZE 字节）"

# --- 校验 yaml ----------------------------------------------------------
step "校验地图文件"

echo "  --- $MAP_NAME.yaml ---"
cat "$YAML" | sed 's/^/    /'

YAML_IMG=$(grep -E '^image:' "$YAML" | awk '{print $2}')
if [ "$YAML_IMG" != "$MAP_NAME.pgm" ]; then
    warn "yaml 里的 image 字段是 '$YAML_IMG'，与文件名不一致"
    warn "正在修正为 $MAP_NAME.pgm（否则 map_server 加载会失败）"
    sed -i "s|^image:.*|image: $MAP_NAME.pgm|" "$YAML"
fi

RES=$(grep -E '^resolution:' "$YAML" | awk '{print $2}')
ORIGIN=$(grep -E '^origin:' "$YAML" | sed 's/^origin: *//')
ok "分辨率 $RES m/像素"
ok "原点   $ORIGIN"

# --- 落盘 ---------------------------------------------------------------
step "写入目标位置"

# 1) 主位置（导航默认读取这里）
cp "$PGM" "$MAPS_DIR/$MAP_NAME.pgm"
cp "$YAML" "$MAPS_DIR/$MAP_NAME.yaml"
ok "已写入 $MAPS_DIR/$MAP_NAME.{pgm,yaml}"

# 2) 历史备份
HIST_DIR="$MAPS_DIR/history"
mkdir -p "$HIST_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
cp "$PGM" "$HIST_DIR/${MAP_NAME}_${STAMP}.pgm"
cp "$YAML" "$HIST_DIR/${MAP_NAME}_${STAMP}.yaml"
ok "已备份 $HIST_DIR/${MAP_NAME}_${STAMP}.{pgm,yaml}"

# 3) 同时存一份到 ~/aic2026/maps（如果存在）
if [ -d "$HOME/aic2026/maps" ]; then
    cp "$PGM" "$HOME/aic2026/maps/$MAP_NAME.pgm"
    cp "$YAML" "$HOME/aic2026/maps/$MAP_NAME.yaml"
    ok "已复制到 ~/aic2026/maps/"
fi

rm -rf "$TMPDIR_MAP"

# --- 完成 ---------------------------------------------------------------
echo
echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"
ok "地图保存完成"
echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"
echo
echo -e "  ${BOLD}用这张地图启动导航：${RESET}"
echo
echo "    roslaunch carebot_nav_kit navigation.launch \\"
echo "        map_file:=$MAPS_DIR/$MAP_NAME.yaml"
echo
echo -e "  ${BOLD}检查地图质量：${RESET}"
echo "    1. 用图像查看器打开 $MAPS_DIR/$MAP_NAME.pgm"
echo "    2. 确认：墙体连续闭合、没有大片黑斑（那是障碍误判）、没有重影"
echo "    3. 若地图扭曲，说明里程计有问题 —— 先跑 odom_monitor.py 检查漂移"
echo
echo -e "  ${BOLD}本包的历史地图：${RESET}"
ls -1t "$HIST_DIR" 2>/dev/null | head -6 | sed 's/^/    /'
echo
