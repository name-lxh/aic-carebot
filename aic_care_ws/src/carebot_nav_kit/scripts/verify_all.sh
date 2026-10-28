#!/bin/bash
# ============================================================================
#  verify_all.sh — 一键完整验证
# ============================================================================
#  在没有机器人、不排队等实车的情况下，验证整套导航代码能否工作。
#
#  依次跑六项：
#    1. 脚本可执行位   —— rosrun 的前提，打包/传输常丢
#    2. catkin 编译    —— 语法与依赖
#    3. launch 解析    —— 引用完整性（ROS 1 用 --nodes，不是 ROS 2 的 --dry-run）
#    4. 纯逻辑单元测试 —— 二维码解析、配置一致性等 58 个用例（离线）
#    5. 配置一致性校验 —— 22 项跨文件检查
#    6. 端到端导航测试 —— 假传感器 + 真实 navigation.launch，真的走到目标点
#
#  第 6 项是关键：它把
#      假传感器 → laser_filter → costmap → amcl → move_base → /cmd_vel
#  整条闭环跑起来，用真值判定是否到达。仅"编译通过"证明不了规划器能用。
#
#  用法：
#      bash verify_all.sh            # 全部
#      bash verify_all.sh --quick    # 跳过端到端（快，约 1 分钟）
#
#  ⚠️ 环境要求：Ubuntu 18.04 + ROS Melodic。
#     ROS Melodic 的核心库只有 python2.7，所以 ROS 相关脚本用 python2 跑；
#     纯离线脚本（单元测试、配置校验）用 python3。
# ============================================================================

set -o pipefail

GREEN='\033[0;32m'; YELLOW='\033[0;33m'; RED='\033[0;31m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
ok()   { echo -e "  ${GREEN}✔${RESET} $*"; }
err()  { echo -e "  ${RED}✘${RESET} $*"; }
warn() { echo -e "  ${YELLOW}⚠${RESET} $*"; }
head1(){ echo -e "\n${CYAN}${BOLD}════════════════════════════════════════════════${RESET}"; \
         echo -e "${CYAN}${BOLD}  $*${RESET}"; \
         echo -e "${CYAN}${BOLD}════════════════════════════════════════════════${RESET}"; }

QUICK=0
[ "$1" = "--quick" ] && QUICK=1

PASS=0
FAIL=0
RESULTS=()

record() {  # record <名称> <0|1>
    if [ "$2" = "0" ]; then
        PASS=$((PASS+1)); RESULTS+=("  ${GREEN}通过${RESET}  $1")
    else
        FAIL=$((FAIL+1)); RESULTS+=("  ${RED}失败${RESET}  $1")
    fi
}

source /opt/ros/melodic/setup.bash 2>/dev/null
cd ~/catkin_ws || { err "找不到 ~/catkin_ws"; exit 2; }
source devel/setup.bash 2>/dev/null

PKG=$(rospack find carebot_nav_kit 2>/dev/null)
if [ -z "$PKG" ]; then
    err "找不到 carebot_nav_kit 包。先编译：cd ~/catkin_ws && catkin_make"
    exit 2
fi
echo "  包路径：$PKG"

# ---------------------------------------------------------------- 1
head1 "1. 脚本可执行位"
bash "$PKG/scripts/fix_permissions.sh" ~/catkin_ws 2>&1 | grep -E '已修复|本来就有|所有节点|仍有'
MISS=$(find src -type f \( -path '*/nodes/*.py' -o -path '*/scripts/*.py' -o -name '*.sh' \) \
       ! -path '*_stubs*' ! -executable 2>/dev/null | wc -l)
if [ "$MISS" = "0" ]; then ok "全部可执行"; record "可执行位" 0
else err "$MISS 个不可执行"; record "可执行位" 1; fi

# ---------------------------------------------------------------- 2
head1 "2. catkin 编译"
catkin_make 2>&1 | tail -3
RC=${PIPESTATUS[0]}
source devel/setup.bash 2>/dev/null
if [ "$RC" = "0" ]; then ok "编译成功"; record "catkin 编译" 0
else err "编译失败"; record "catkin 编译" 1; fi

# ---------------------------------------------------------------- 3
head1 "3. launch 解析（ROS 1 用 --nodes）"
LFAIL=0
for f in bringup mapping navigation; do
    OUT=$(timeout 45 roslaunch --nodes carebot_nav_kit "$f.launch" 2>&1)
    if [ $? -eq 0 ]; then
        ok "$f.launch（$(echo "$OUT" | grep -c '^/') 节点）"
    else
        err "$f.launch 解析失败"
        echo "$OUT" | head -3 | sed 's/^/      /'
        LFAIL=1
    fi
done
record "launch 解析" $LFAIL

# ---------------------------------------------------------------- 4
head1 "4. 纯逻辑单元测试（离线，58 个用例）"
cd "$PKG/test" || exit 2
UT=$(timeout 180 python3 test_pure_logic.py 2>&1)
echo "$UT" | tail -5
if echo "$UT" | grep -q '失败，0 错误'; then UTRC=0; else UTRC=1; fi
record "单元测试" $UTRC
cd ~/catkin_ws

# ---------------------------------------------------------------- 5
head1 "5. 配置一致性校验（22 项）"
CF=$(timeout 120 python3 "$PKG/scripts/verify_nav_config.py" 2>&1)
echo "$CF" | tail -4
if echo "$CF" | grep -q '失败 0 项'; then CFRC=0; else CFRC=1; fi
record "配置一致性" $CFRC

# ---------------------------------------------------------------- 6
if [ "$QUICK" = "1" ]; then
    head1 "6. 端到端导航测试（已跳过 --quick）"
else
    head1 "6. 端到端导航测试（假传感器 + 真实 navigation.launch）"
    pkill -f rosmaster 2>/dev/null
    pkill -f roslaunch 2>/dev/null
    pkill -f rosrun 2>/dev/null
    sleep 3
    cd "$PKG/test" || exit 2
    E2E_OUT=$(timeout 480 python2 e2e_nav_test.py 2>&1)
    echo "$E2E_OUT" | tail -20
    if echo "$E2E_OUT" | grep -q '导航闭环完全正常'; then E2ERC=0; else E2ERC=1; fi
    record "端到端导航" $E2ERC
    cd ~/catkin_ws
fi

# ---------------------------------------------------------------- 汇总
echo
echo -e "${BOLD}════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}  验证汇总${RESET}"
echo -e "${BOLD}════════════════════════════════════════════════${RESET}"
for r in "${RESULTS[@]}"; do echo -e "$r"; done
echo
echo -e "  通过 ${GREEN}$PASS${RESET}   失败 ${RED}$FAIL${RESET}"
echo -e "${BOLD}════════════════════════════════════════════════${RESET}"

if [ "$FAIL" = "0" ]; then exit 0; else exit 1; fi
