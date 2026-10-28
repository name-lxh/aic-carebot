#!/bin/bash
# ============================================================================
#  voice_check.sh —— 语音链路一键自检（发车之前跑，30 秒出结论）
# ============================================================================
#  为什么要有它：语音是这条链路里最容易"看起来起来了、其实没声/没麦"的一环。
#  今晚踩过的三次坑，本脚本逐项验证：
#    1) TTS 没声：~/.asoundrc 默认声卡指错（车上是按卡名绑的 PCH）
#    2) ASR 没麦：VAD 条数为 0（相机抢麦/节点没起来）
#    3) 阈值太严：真识别到了但 confidence 低于阈值被丢
#
#  用法： bash ~/Hajimi/LXH/_tools/voice_check.sh
#  退出码：0 = 全过；非 0 = 有项目没过（看输出）
# ============================================================================
set -u
LXH=/home/ub/Hajimi/LXH
TOOLS=$LXH/_tools
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
export PYTHONIOENCODING=utf-8
export PYTHONPATH=$TOOLS
source "$LXH/env.sh" >/dev/null 2>&1

FAIL=0
ok()   { echo "  ✅ $1"; }
bad()  { echo "  ❌ $1"; FAIL=$((FAIL+1)); }
warn() { echo "  ⚠  $1"; }

echo "════ 1. ROS master（11399）════"
if ss -ltn 2>/dev/null | grep -q ':11399'; then ok "master 在线"; else
    bad "master 不在 —— 先 bash ~/start.sh --no-mission 或 roslaunch 起栈"; fi

echo "════ 2. TTS 节点 ════"
if pgrep -f "tts_ne[t][.]py" >/dev/null; then ok "tts_net 在跑 (pid=$(pgrep -f 'tts_ne[t][.]py' | head -1))"; else
    bad "tts_net 没跑（/voice/say 没人接）"; fi

echo "════ 3. 声卡路由与音量 ════"
DVOL=$(amixer -c PCH sget Master 2>/dev/null | tail -2 | head -1 | sed -e 's/.*\[\([0-9]*%\)\].*/\1/')
if [ -n "$DVOL" ]; then ok "PCH Master = $DVOL（建议 70% 以上）"; else warn "读不到 PCH Master 音量"; fi
if grep -q 'CARD=PCH' ~/.asoundrc 2>/dev/null; then ok "默认播放设备已按卡名绑到 PCH"; else
    warn "~/.asoundrc 未绑定 CARD=PCH（卡号一变就没声）"; fi

echo "════ 4. 放一段提示音（应当听到）════"
python2 - <<'PY' >/tmp/voice_check_beep.log 2>&1
import struct, wave
f, sec, rate = 1000.0, 1.0, 48000
n = int(sec * rate)
data = b''.join(struct.pack('<h', int(16000 * (1 if (i * f / rate) % 1 < 0.5 else -1))) for i in range(n))
w = wave.open('/tmp/voice_check_beep.wav', 'wb'); w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
w.writeframes(data); w.close()
PY
timeout 6 aplay /tmp/voice_check_beep.wav >/dev/null 2>&1 && ok "提示音已播放" || bad "提示音播放失败（声卡/接线）"

echo "════ 5. TTS 播报 ════"
timeout 5 rostopic pub -1 /voice/say std_msgs/String "data: \"语音自检，请说查询室内温度\"" >/dev/null 2>&1
sleep 3
if tail -6 /tmp/f_tts.log 2>/dev/null | grep -q "播放"; then ok "TTS 已合成并播放"; else
    warn "没看到 TTS 播放日志（检查 /tmp/f_tts.log）"; fi

echo "════ 6. ASR 麦克风（VAD）════"
if pgrep -f "voice_contro[l]_usb" >/dev/null; then
    VAD=$(grep -ac '\[VAD\]' /tmp/asr.log 2>/dev/null)
    if [ "${VAD:-0}" -gt 0 ]; then ok "ASR 在跑，VAD 条数 $VAD"; else
        bad "ASR 在跑但 VAD=0（麦克风没数据）"
        pgrep -x pulseaudio >/dev/null && echo "     ↳ 发现 pulseaudio 占着麦克风：pulseaudio --kill 后重启 ASR"
        grep -a "Device or resource busy" /tmp/asr.log >/dev/null 2>&1 && echo "     ↳ asr.log 里有 Device busy：同上"
    fi
    grep -a "解析结果" /tmp/asr.log | tail -1 | sed -e 's/^/     最近一次识别: /'
else
    bad "ASR 没跑（bash $TOOLS/asr_bg.sh）"
fi

echo "════ 7. 识别一句话 ════"
timeout 20 python2 -u "$TOOLS/voice_probe.py" --timeout 12 || FAIL=$((FAIL+1))

echo ""
if [ "$FAIL" = "0" ]; then
    echo "════ 结论：语音链路全绿 ════"
    exit 0
else
    echo "════ 结论：有 $FAIL 项没过，先修再发车 ════"
    exit 1
fi
