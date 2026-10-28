#!/bin/bash
# asr_boot.sh —— 重启后专用：只起 master + ASR（不启相机），验证音频流
#   相机和 ASR 抢 Astra 的音频设备，语音测试阶段先不启相机。
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
VENDOR=/home/ub/omniWheelCareRobot/rosCode/src/xf_mic_asr_offline

echo "════ 1. master ════"
if ! ss -ltn 2>/dev/null | grep -q ':11399'; then
    setsid nohup roscore -p 11399 > /tmp/asr_roscore.log 2>&1 < /dev/null &
    for i in $(seq 1 60); do ss -ltn 2>/dev/null | grep -q ':11399' && break; sleep 0.2; done
fi
ss -ltn 2>/dev/null | grep -q ':11399' && echo "  OK" || { echo "  FAIL"; exit 1; }

echo "════ 2. 词表 + 增益 ════"
if [ -f "$LXH/_tools/vocab/call.bnf" ]; then
    cp "$LXH/_tools/vocab/call.bnf" "$VENDOR/config/call.bnf"
    echo "  词表已安装"
fi
amixer -c 0 sset Mic,0 60% >/dev/null 2>&1
amixer -c 0 sset Mic,1 60% >/dev/null 2>&1

echo "════ 3. ASR ════"
for pat in xf_asr_usb_node command_recognition motion_control; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
done
sleep 2
( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
  setsid nohup roslaunch xf_mic_asr_offline voice_init.launch > /tmp/asr.log 2>&1 < /dev/null & )
sleep 11
echo "  进程: $(pgrep -f xf_asr_usb_node | head -1)"
echo "  设备: $(sed 's/\x1b\[[0-9;]*m//g' /tmp/asr.log 2>/dev/null | grep -a 录音设备 | head -1)"
echo "  登录: $(sed 's/\x1b\[[0-9;]*m//g' /tmp/asr.log 2>/dev/null | grep -a MSPLogin | head -1)"
echo "  文法: $(sed 's/\x1b\[[0-9;]*m//g' /tmp/asr.log 2>/dev/null | grep -a QISRBuildGrammar | head -1)"
echo "  音频: $(tail -60 /tmp/asr.log 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | grep -c AudioWrite) 条 AudioWrite"
