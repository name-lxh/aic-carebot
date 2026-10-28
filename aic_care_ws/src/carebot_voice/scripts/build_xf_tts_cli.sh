#!/bin/bash
# ============================================================================
#  build_xf_tts_cli.sh — 编译一次性讯飞离线 TTS 命令行工具
# ============================================================================
#  用法：
#      ./build_xf_tts_cli.sh <robot_voice包路径> [输出路径]
#
#  示例：
#      ./build_xf_tts_cli.sh ~/omniWheelCareRobot/rosCode/src/robot_voice
#      ./build_xf_tts_cli.sh ~/omniWheelCareRobot/rosCode/src/robot_voice /usr/local/bin
#
#  它会链接 robot_voice 包里的 libmsc.so 与头文件。
#  编译成功后，把输出路径配到 voice_bridge 的 ~tts_bin 参数。
# ============================================================================

set -e

PKG_DIR="${1:-}"
OUT_DIR="${2:-}"

if [ -z "$PKG_DIR" ]; then
    echo "用法: $0 <robot_voice包路径> [输出路径]" >&2
    echo "示例: $0 ~/omniWheelCareRobot/rosCode/src/robot_voice" >&2
    exit 2
fi

# --- 定位包内资源 -----------------------------------------------------------
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ ! -d "$PKG_DIR" ]; then
    echo "[错误] 找不到 robot_voice 包目录: $PKG_DIR" >&2
    exit 2
fi

INC_DIR="$PKG_DIR/include/robot_voice"
LIB_DIR="$PKG_DIR/libs"

if [ ! -d "$INC_DIR" ]; then
    echo "[错误] 找不到头文件目录: $INC_DIR" >&2
    echo "       期望 robot_voice 包内包含 include/robot_voice/*.h" >&2
    exit 2
fi

if [ ! -f "$LIB_DIR/libmsc.so" ]; then
    echo "[错误] 找不到 $LIB_DIR/libmsc.so" >&2
    echo "       请确认 robot_voice 包完整（libs/ 下应有 libmsc.so / libhid_lib.so 等）" >&2
    exit 2
fi

# --- 输出位置 ---------------------------------------------------------------
if [ -n "$OUT_DIR" ]; then
    mkdir -p "$OUT_DIR"
    TARGET="$OUT_DIR/xf_tts_cli"
else
    TARGET="$SRC_DIR/xf_tts_cli"
fi

echo "头文件目录 : $INC_DIR"
echo "库目录     : $LIB_DIR"
echo "输出       : $TARGET"
echo "链接库     : libmsc.so -ldl -lpthread -lm -lrt"

# --- 编译 -------------------------------------------------------------------
g++ -O2 -std=c++11 \
    "$SRC_DIR/xf_tts_cli.cpp" \
    -I"$INC_DIR" \
    -L"$LIB_DIR" \
    -lmsc -ldl -lpthread -lm -lrt \
    -Wl,-rpath,"$LIB_DIR" \
    -o "$TARGET"

chmod +x "$TARGET"
echo
echo "[成功] 已生成: $TARGET"
echo

# --- 自检 -------------------------------------------------------------------
echo "--- 自检：尝试合成一句测试语音 ---"
TEST_WAV="${TMPDIR:-/tmp}/xf_tts_cli_selftest.wav"
if "$TARGET" "语音合成自检成功" "$TEST_WAV"; then
    if [ -s "$TEST_WAV" ]; then
        echo "[成功] 已生成测试语音: $TEST_WAV ($(stat -c%s "$TEST_WAV") 字节)"
        echo "       播放验证: aplay -q $TEST_WAV"
    else
        echo "[警告] 合成进程返回 0 但输出文件为空，请检查 libmsc.so 与 appid 是否匹配"
    fi
else
    echo "[警告] 合成自检失败（可能是 appid 与 libmsc.so 不绑定，或授权已过期）"
    echo "       兜底方案：改用 voice_bridge 的 espeak 备用合成器，或使用资料包中的"
    echo "       「动态库-不与ID绑定」替换库。"
fi

echo
echo "--- 下一步 ---"
echo "把下面这行加入你的 launch 文件（或直接传给 voice_bridge）："
echo "  <param name=\"tts_bin\" value=\"$TARGET\"/>"
