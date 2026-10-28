#!/bin/bash
# asr_bg.sh —— 后台起 ASR，等足够久（20 s）再验证（节点初始化要 12~15 s）
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
export PYTHONUNBUFFERED=1
VENDOR=/home/ub/omniWheelCareRobot/rosCode/src/xf_mic_asr_offline
TOOLS=/home/ub/Hajimi/LXH/_tools

if [ -f "$TOOLS/vocab/call.bnf" ]; then
    cp "$TOOLS/vocab/call.bnf" "$VENDOR/config/call.bnf"
    echo "  词表已安装（我们的 call.bnf）"
fi
if [ -f "$TOOLS/vocab/appid_params.yaml" ]; then
    cp "$TOOLS/vocab/appid_params.yaml" "$VENDOR/config/appid_params.yaml"
    echo "  appid_params 已安装（confidence=$(grep -m1 confidence "$TOOLS/vocab/appid_params.yaml" | awk '{print $2}')）"
fi
if [ -f "$TOOLS/vocab/recognition_params.yaml" ]; then
    cp "$TOOLS/vocab/recognition_params.yaml" "$VENDOR/config/recognition_params.yaml"
    echo "  识别参数已安装（confidence=5：现场正确识别只有 8 分，15 会丢掉）"
fi
# 麦克风增益：2026-10-23 现场环境嘈杂、拾音太敏感，60% -> 45%（可用 MIC_GAIN 覆盖）。
# 声卡编号会变（Astra 有时是 card 0、有时不是），按名字 S 找，找不到再退回 0。
MIC_GAIN=${MIC_GAIN:-60}   # V100 的值；想压噪声再临时 export MIC_GAIN=45
for _c in S 0; do
    if amixer -c "$_c" sset Mic,0 "${MIC_GAIN}%" >/dev/null 2>&1; then
        amixer -c "$_c" sset Mic,1 "${MIC_GAIN}%" >/dev/null 2>&1
        echo "  麦克风增益 ${MIC_GAIN}%（card $_c）"
        break
    fi
done

# ★ 2026-10-23：PulseAudio 会抢占相机麦克风（pcmC1D0c），ASR 一开设备就
#   "Device or resource busy"，VAD 恒为 0、说了没反应。这台车音频全走 ALSA 直连，
#   pulse 没有用处，起 ASR 前先关掉（下次登录若自动拉起，这里也会再关一次）。
if pgrep -x pulseaudio >/dev/null 2>&1; then
    pulseaudio --kill 2>/dev/null
    sleep 1
    echo "  已停止 pulseaudio（它占着麦克风）"
fi

for p in $(pgrep -f "xf_asr_usb_node" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "command_recognition" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 2

rm -f /tmp/asr.log
( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
  setsid nohup stdbuf -oL roslaunch xf_mic_asr_offline voice_init.launch > /tmp/asr.log 2>&1 < /dev/null & )

# 节点初始化要 12~15 s，这里等 22 s 再判定
sleep 22
echo "  进程: $(pgrep -f xf_asr_usb_node | head -1)"
echo "  登录: $(grep -a MSPLogin /tmp/asr.log 2>/dev/null | head -1)"
echo "  文法: $(grep -a QISRBuildGrammar /tmp/asr.log 2>/dev/null | head -1)"
echo "  设备: $(grep -a 录音设备 /tmp/asr.log 2>/dev/null | head -1)"
VAD=$(grep -ac '\[VAD\]' /tmp/asr.log 2>/dev/null)
if [ "$VAD" -eq 0 ]; then
    echo "  [warn] VAD=0（麦克风没数据），重启一次语音引擎…"
    for p in $(pgrep -f "xf_asr_usb_node" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
    sleep 4
    rm -f /tmp/asr.log
    ( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
      setsid nohup stdbuf -oL roslaunch xf_mic_asr_offline voice_init.launch > /tmp/asr.log 2>&1 < /dev/null & )
    sleep 22
    VAD=$(grep -ac '\[VAD\]' /tmp/asr.log 2>/dev/null)
fi
echo "  VAD 条数: $VAD"

# TTS（播报用）
if ! pgrep -f "tts_net" >/dev/null 2>&1; then
    ( setsid nohup python2 -u "$TOOLS/tts_net.py" > /tmp/tts_asr.log 2>&1 < /dev/null & )
    sleep 2
fi
echo "  tts: $(pgrep -f tts_net | head -1)"
