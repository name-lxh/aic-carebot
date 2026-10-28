#!/bin/bash
# asr_fg_debug.sh —— 前台起一次 ASR，抓前 20 行日志（诊断用）
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null

for p in $(pgrep -f "xf_asr_usb_node" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f "command_recognition" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 2

timeout 16 roslaunch xf_mic_asr_offline voice_init.launch 2>&1 \
    | grep -avE "disk usage|rosclean|UnicodeWarning|resource_name in files" \
    | head -20
