#!/usr/bin/env bash
# ============================================================================
#  fix_permissions.sh — 给工作空间里所有需要执行的脚本加可执行位
# ============================================================================
#  为什么需要这个
# --------------
#  实测发现：从 Windows 传到 Linux 的脚本全部是 -rw-rw-r--（无可执行位）。
#  后果：rosrun 报
#      [rosrun] Found the following, but they're either not files, or not executable
#  所有节点都起不来。
#
#  这个问题在**打包成镜像**时同样会出现（tar/zip 传输、git clone 都可能丢权限），
#  所以修复脚本要保留在工程里，部署后跑一次。
#
#  同时写一个 .gitattributes 让 git 记住可执行位：
#      *.sh  text eol=lf
#      *.py  text eol=lf
#  配合 `git update-index --chmod=+x`（如果需要）。
#
#  用法：
#      bash fix_permissions.sh                    # 修复默认工作空间
#      bash fix_permissions.sh ~/catkin_ws        # 指定工作空间
# ============================================================================

set -o pipefail

GREEN='\033[0;32m'; YELLOW='\033[0;33m'; RED='\033[0;31m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
ok()   { echo -e "  ${GREEN}✔${RESET} $*"; }
warn() { echo -e "  ${YELLOW}⚠${RESET} $*"; }
err()  { echo -e "  ${RED}✘${RESET} $*"; }

WS="${1:-$HOME/catkin_ws}"
SRC="$WS/src"

echo -e "${BOLD}"
echo "════════════════════════════════════════════════════════════════"
echo "  修复脚本可执行位"
echo "  工作空间：$WS"
echo "════════════════════════════════════════════════════════════════"
echo -e "${RESET}"

if [ ! -d "$SRC" ]; then
    err "找不到 $SRC"
    exit 1
fi

FIXED=0
ALREADY=0
SKIPPED=0

# ---------------------------------------------------------------------------
# 规则：
#   nodes/*.py            必须有 +x（rosrun 直接执行）
#   scripts/*.py          必须有 +x（rosrun 执行）
#   scripts/*.sh          必须有 +x
#   *.sh 在包根或任意位置 必须有 +x
#   launch / config 不用
# ---------------------------------------------------------------------------
fix_one() {
    local f="$1"
    if [ -x "$f" ]; then
        ALREADY=$((ALREADY+1))
        return
    fi
    if chmod +x "$f" 2>/dev/null; then
        FIXED=$((FIXED+1))
        echo "    +x  ${f#$SRC/}"
    else
        err "  chmod 失败：$f"
        SKIPPED=$((SKIPPED+1))
    fi
}

echo
echo "▶ 扫描并修复"

# nodes 与 scripts 下的 .py
while IFS= read -r f; do
    [ -n "$f" ] && fix_one "$f"
done < <(find "$SRC" -type f -path '*/nodes/*.py' 2>/dev/null)

while IFS= read -r f; do
    [ -n "$f" ] && fix_one "$f"
done < <(find "$SRC" -type f -path '*/scripts/*.py' 2>/dev/null)

# 所有 .sh
while IFS= read -r f; do
    [ -n "$f" ] && fix_one "$f"
done < <(find "$SRC" -type f -name '*.sh' 2>/dev/null)

# CMakeLists 里 catkin_install_python 列出的文件（双保险）
while IFS= read -r cml; do
    pkgdir=$(dirname "$cml")
    # 提取 catkin_install_python(...) 块里的路径
    awk '/catkin_install_python\(/,/\)/' "$cml" 2>/dev/null | \
        grep -oE '[a-zA-Z0-9_/]+\.(py|sh)' | while read -r rel; do
            f="$pkgdir/$rel"
            [ -f "$f" ] && fix_one "$f"
        done
done < <(find "$SRC" -name CMakeLists.txt 2>/dev/null)

# 包根目录的 setup.py（ament 包用，ROS 1 一般不需要，但加上无害）
while IFS= read -r f; do
    [ -n "$f" ] && fix_one "$f"
done < <(find "$SRC" -maxdepth 2 -name 'setup.py' -type f 2>/dev/null)

echo
echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"
echo "  已修复 : $FIXED 个"
echo "  本来就有: $ALREADY 个"
[ "$SKIPPED" -gt 0 ] && echo -e "  ${RED}失败   : $SKIPPED 个${RESET}"
echo -e "${BOLD}════════════════════════════════════════════════════════════════${RESET}"

# ---------------------------------------------------------------------------
# 验证：列出仍然不可执行的候选
# ---------------------------------------------------------------------------
echo
echo "▶ 验证"
REMAIN=$(find "$SRC" -type f \( -path '*/nodes/*.py' -o -path '*/scripts/*.py' -o -name '*.sh' \) ! -executable 2>/dev/null | wc -l)
if [ "$REMAIN" -eq 0 ]; then
    ok "所有节点/脚本都可执行"
else
    err "仍有 $REMAIN 个不可执行："
    find "$SRC" -type f \( -path '*/nodes/*.py' -o -path '*/scripts/*.py' -o -name '*.sh' \) ! -executable 2>/dev/null | sed 's/^/    /'
fi

# ---------------------------------------------------------------------------
# 顺便写 .gitattributes，避免 git 丢权限 / 改换行符
# ---------------------------------------------------------------------------
ATTR="$SRC/.gitattributes"
if [ ! -f "$ATTR" ]; then
    cat > "$ATTR" <<'EOF'
# 保证脚本在 checkout 后仍有可执行位，且不会被改成 CRLF
# （CRLF 的 shell 脚本在 Linux 上会报 "bad interpreter: ^M"）
*.sh   text eol=lf
*.py   text eol=lf
*.launch text eol=lf
*.yaml text eol=lf
*.md   text eol=lf
EOF
    ok "已写入 $ATTR（防止打包/传输丢失权限与换行符）"
fi

echo
echo "  下一步："
echo "    source $WS/devel/setup.bash"
echo "    rosrun carebot_nav_kit nav_check.py --help    # 验证能跑"
