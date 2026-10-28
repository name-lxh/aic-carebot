#!/bin/bash
# ============================================================================
#  grab_screen.sh —— 抓取机器人当前屏幕（证明桌面/GUI 真的活着）
# ============================================================================
#  ⚠️ 为什么要抓屏而不是只测端口：
#     "端口开着" ≠ "VNC 能用"。端口开着但服务卡死、桌面黑屏、
#     没有图形会话的情况都很常见。抓一张真实屏幕是**最硬的验证** ——
#     能直接看出桌面是不是真的在跑、分辨率多少、有没有程序在运行。
#
#  ⚠️ 只用最温和的方式：import/xwd 都是**只读**抓取，不影响任何会话，
#     不打断别人的 VNC 连接。
#
#  屏幕分辨率实测 1024x600 —— 正是指导书"事项二"提到的分辨率太小的问题。
# ============================================================================
set -u

OUT=/tmp/robot_screen.png
export DISPLAY=:0

echo "════════ 1. 图形会话 ════════"
echo "  X sockets: $(ls /tmp/.X11-unix/ 2>/dev/null | tr '\n' ' ')"
echo "  会话:"
who 2>/dev/null | sed 's/^/      /'

echo ""
echo "════════ 2. 屏幕分辨率 ════════"
timeout 10 xdpyinfo 2>/dev/null | grep -E 'dimensions|depth of root' \
    | sed 's/^/  /'

echo ""
echo "════════ 3. 抓屏 ════════"
rm -f "$OUT" /tmp/robot_screen.xwd 2>/dev/null

if command -v import >/dev/null 2>&1; then
    echo "  用 ImageMagick import（-window root，只读）"
    timeout 40 import -window root "$OUT" 2>&1 | head -5 | sed 's/^/      /'
fi

if [ ! -s "$OUT" ]; then
    echo "  import 没成功，改用 xwd"
    timeout 40 xwd -root -silent > /tmp/robot_screen.xwd 2>/dev/null
    if [ -s /tmp/robot_screen.xwd ]; then
        command -v convert >/dev/null 2>&1 && \
            timeout 40 convert /tmp/robot_screen.xwd "$OUT" 2>/dev/null
        [ -s "$OUT" ] || cp /tmp/robot_screen.xwd "$OUT"
    fi
fi

if [ -s "$OUT" ]; then
    echo "  ✔ 已抓取: $OUT"
    ls -la "$OUT" | sed 's/^/      /'
    file "$OUT" 2>/dev/null | sed 's/^/      /'
else
    echo "  ✘ ★ 抓屏失败"
fi

echo ""
echo "════════ 4. 屏幕统计（判断是不是黑屏）════════"
if [ -s "$OUT" ] && command -v convert >/dev/null 2>&1; then
    echo "  平均亮度/颜色数:"
    timeout 40 convert "$OUT" -format '    平均色 %[mean]  颜色数 %k  尺寸 %wx%h' info: 2>/dev/null
    echo ""
    echo "  直方图前 8 个颜色:"
    timeout 40 convert "$OUT" -format %c -depth 8 histogram:info: 2>/dev/null \
        | sort -rn | head -8 | sed 's/^/      /'
fi

echo ""
echo "════════ 5. 完成 ════════"
echo "  文件: $OUT"
