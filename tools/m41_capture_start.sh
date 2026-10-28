#!/bin/bash
# m41_capture_start.sh —— 一键：确保 master/相机/TTS 正常，然后开始采人脸样本
#   用法: bash m41_capture_start.sh [张数] [秒数]
set -u
LXH=/home/ub/Hajimi/LXH
OUT=$LXH/face_dataset/id2_raw
WANT=${1:-100}
TOTAL=${2:-300}
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
export PYTHONIOENCODING=utf-8
source "$LXH/env.sh" 2>/dev/null

echo "════ 1. master ════"
if ! ss -ltn 2>/dev/null | grep -q ':11399'; then
    setsid nohup roscore -p 11399 > /tmp/cap_roscore.log 2>&1 < /dev/null &
    for i in $(seq 1 50); do ss -ltn 2>/dev/null | grep -q ':11399' && break; sleep 0.2; done
fi
ss -ltn 2>/dev/null | grep -q ':11399' && echo "  OK" || { echo "  FAIL"; exit 1; }

echo "════ 2. 相机 ════"
HZ=$(timeout 6 rostopic hz /camera/rgb/image_raw 2>&1 | grep -m1 average)
if [ -z "$HZ" ]; then
    for p in $(pgrep -f "astra.launch" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
    for p in $(pgrep -f "camera_nodelet_manager" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
    for p in $(pgrep -f "nodelet" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
    sleep 3
    ( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
      setsid nohup roslaunch astra_launch astra.launch > /tmp/cap_astra.log 2>&1 < /dev/null & )
    sleep 13
    HZ=$(timeout 8 rostopic hz /camera/rgb/image_raw 2>&1 | grep -m1 average)
fi
echo "  ${HZ:-★无数据}"
[ -n "$HZ" ] || { echo "  相机起不来，退出"; exit 2; }

echo "════ 3. TTS ════"
if ! pgrep -f "tts_net" >/dev/null 2>&1; then
    ( setsid nohup python2 -u "$LXH/_tools/tts_net.py" > /tmp/cap_tts.log 2>&1 < /dev/null & )
    sleep 3
fi
echo "  tts pid: $(pgrep -f tts_net | head -1)"

echo "════ 4. 开始采样 ════"
rm -rf "$OUT"; mkdir -p "$OUT"
setsid nohup python2 -u "$LXH/_tools/capture_face_dataset.py" "$OUT" "$WANT" "$TOTAL" \
    > /tmp/capture.log 2>&1 < /dev/null &
sleep 6
echo "  capture pid: $(pgrep -f capture_face_dataset | head -1)"
tail -3 /tmp/capture.log
