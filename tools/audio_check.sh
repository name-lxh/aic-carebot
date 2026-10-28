#!/bin/bash
# audio_check.sh —— 查声卡/默认设备/播放器配置，并把 mpv 指到模拟输出
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213

echo "=== 停止采样进程 ==="
for p in $(pgrep -f capture_face_datas 2>/dev/null; pgrep -f "capture_face" 2>/dev/null); do
    [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null
done
echo "  done"

echo "=== 声卡列表 ==="
cat /proc/asound/cards 2>/dev/null | grep -E "^ *[0-9]" | head -5

echo "=== ALSA 设备（前几个）==="
aplay -L 2>/dev/null | grep -E "^[a-z]" | head -8

echo "=== asound.conf ==="
cat /etc/asound.conf 2>/dev/null | head -8
cat "$HOME/.asoundrc" 2>/dev/null | head -8

echo "=== tts_net 播放器 ==="
grep -nE "PLAYER" /home/ub/Hajimi/LXH/_tools/tts_net.py | head -3

echo "=== 试播（mpv，模拟口 card2 dev0）==="
ls /usr/share/sounds/alsa/*.wav 2>/dev/null | head -1
F=$(ls /usr/share/sounds/alsa/Front_Center.wav 2>/dev/null | head -1)
if [ -n "$F" ]; then
    timeout 10 mpv --no-video --really-quiet --volume=60 --audio-device=alsa/plughw:2,0 "$F" 2>&1 | tail -3
    echo "  mpv 退出码 $?"
else
    echo "  没有测试音频"
fi
