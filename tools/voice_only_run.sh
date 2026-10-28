#!/bin/bash
# voice_only_run.sh —— 车放在 A 点时，只跑"语音交互"这一段
#   1) 起完整栈（导航/iot/TTS）
#   2) 位姿设为 A 点（车实际就在 A）
#   3) 停相机释放麦克风 -> 起 ASR
#   4) 跑 voice_dialog 三轮对话
set -u
LXH=/home/ub/Hajimi/LXH
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
export PYTHONIOENCODING=utf-8
source "$LXH/env.sh" 2>/dev/null

echo "════ 1. 起完整栈 ════"
bash "$LXH/_tools/m40_fast_start.sh" 2>&1 | tail -5

echo "════ 2. 车在起点：定位后先开到 A 点 ════"
cd "$LXH/_tools" && timeout 35 python2 set_init_pose.py 0 0 0 2>&1 | grep converged
sleep 2
cd "$LXH/_tools" && timeout 120 python2 -u nav_to_a.py 2>&1 | grep -vE "RUNNING" | tail -4

echo "════ 3. 停相机 + 起 ASR ════"
for p in $(pgrep -f "astra.launch" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "camera_nodelet_manager" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "nodelet" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 4
bash "$LXH/_tools/asr_bg.sh" 2>&1 | tail -5

echo "════ 4. 服务检查 ════"
for s in /iot/refresh /iot/control /face/verify; do
    printf "  %-16s " "$s"
    timeout 5 rosservice list 2>/dev/null | grep -qx "$s" && echo OK || echo 缺
done
printf "  %-16s " "/voice_words"
timeout 5 rostopic list 2>/dev/null | grep -qx /voice_words && echo OK || echo 缺

echo "════ 5. 开始三轮语音交互 ════"
cd "$LXH/_tools" && timeout 300 python2 -u voice_dialog.py 2>&1 | tail -40
