#!/bin/bash
# voice_only_nomove.sh —— 只测语音交互，**不移动小车**
#   需要：master + iot（读温度/控设备）+ TTS + ASR
#   不需要：相机、导航、move_base
set -u
LXH=/home/ub/Hajimi/LXH
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
export PYTHONIOENCODING=utf-8
source "$LXH/env.sh" 2>/dev/null

echo "════ 1. master ════"
if ! ss -ltn 2>/dev/null | grep -q ':11399'; then
    setsid nohup roscore -p 11399 > /tmp/nm_roscore.log 2>&1 < /dev/null &
    for i in $(seq 1 60); do ss -ltn 2>/dev/null | grep -q ':11399' && break; sleep 0.2; done
fi
ss -ltn 2>/dev/null | grep -q ':11399' && echo "  OK" || { echo "  FAIL"; exit 1; }

echo "════ 2. IOT 桥接（读温度 / 控设备）════"
if ! timeout 5 rosservice list 2>/dev/null | grep -qx /iot/refresh; then
    setsid nohup roslaunch carebot_iot iot.launch token_file:=$LXH/.ha_token \
        > /tmp/nm_iot.log 2>&1 < /dev/null &
    sleep 9
fi
printf "  /iot/refresh "; timeout 5 rosservice list 2>/dev/null | grep -qx /iot/refresh && echo OK || echo 缺
printf "  /iot/control "; timeout 5 rosservice list 2>/dev/null | grep -qx /iot/control && echo OK || echo 缺

echo "════ 3. TTS ════"
if ! pgrep -f "tts_net" >/dev/null 2>&1; then
    ( setsid nohup python2 -u "$LXH/_tools/tts_net.py" > /tmp/nm_tts.log 2>&1 < /dev/null & )
    sleep 3
fi
echo "  tts: $(pgrep -f tts_net | head -1)"

echo "════ 4. ASR ════"
bash "$LXH/_tools/asr_bg.sh" 2>&1 | tail -5

echo "════ 5. 三轮语音交互（不动车）════"
cd "$LXH/_tools" && timeout 260 python2 -u voice_dialog.py 2>&1 | tail -40
