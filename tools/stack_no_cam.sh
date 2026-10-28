#!/bin/bash
# stack_no_cam.sh —— 起完整栈（导航 + iot + TTS），但**不起/不保留相机**（让麦克风给 ASR）
set -u
LXH=/home/ub/Hajimi/LXH
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
source "$LXH/env.sh" 2>/dev/null

echo "════ 1. 完整栈（m40，不含 mission）════"
bash "$LXH/_tools/m40_fast_start.sh" 2>&1 | tail -6

echo "════ 2. 停相机（释放 Astra 麦克风）════"
for p in $(pgrep -f "astra.launch" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "camera_nodelet_manager" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "nodelet" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 4
echo "  相机进程: $(pgrep -fc 'camera_nodelet_manager' 2>/dev/null || echo 0)"

echo "════ 3. ASR ════"
bash "$LXH/_tools/asr_bg.sh" 2>&1 | tail -6

echo "════ 4. 服务复查 ════"
for s in /iot/refresh /iot/control; do
    printf "  %-16s " "$s"
    timeout 6 rosservice list 2>/dev/null | grep -qx "$s" && echo OK || echo 缺
done
printf "  %-16s " "move_base"
timeout 6 rostopic hz /move_base/status 2>&1 | grep -m1 -q average && echo OK || echo 缺
printf "  %-16s " "/voice_words"
timeout 6 rostopic list 2>/dev/null | grep -qx /voice_words && echo OK || echo 缺
