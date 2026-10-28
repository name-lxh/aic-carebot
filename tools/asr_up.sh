#!/bin/bash
# asr_up.sh —— 起相机（提供 Astra 麦克风）+ 讯飞离线 ASR + 在线 TTS
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213

# 1) 相机：Astra 的麦克风挂在相机上，相机不起就没有录音设备
if ! pgrep -f camera_nodelet_manager >/dev/null 2>&1; then
    ( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
      setsid nohup roslaunch astra_launch astra.launch > /tmp/asr_astra.log 2>&1 < /dev/null & )
    sleep 12
fi
CAM=$(timeout 6 rostopic hz /camera/rgb/image_raw 2>&1 | grep -m1 -E "average|no new")
echo "  相机: ${CAM:-无}"

# 1.5) 安装我们自己的词表（可迁移：词表放在 _tools/real_test/vocab/）
VENDOR=/home/ub/omniWheelCareRobot/rosCode/src/xf_mic_asr_offline
if [ -f "$LXH/_tools/vocab/call.bnf" ]; then
    cp "$LXH/_tools/vocab/call.bnf" "$VENDOR/config/call.bnf"
    echo "  词表已安装（vocab/call.bnf -> 厂商 config）"
fi
# 采集增益 60%：100% 会削顶（peak 32768），引擎直接返回空
amixer -c 0 sset Mic,0 60% >/dev/null 2>&1
amixer -c 0 sset Mic,1 60% >/dev/null 2>&1

# 2) ASR（**彻底清干净再起**，否则同名节点互踢：
#    "Shutdown request received. Reason: [new node registered with same name]"）
for pat in xf_asr_usb_node command_recognition motion_control; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
done
for p in $(pgrep -f "xf_mic_asr_offline/launch" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 3
( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
  setsid nohup roslaunch xf_mic_asr_offline voice_init.launch > /tmp/asr.log 2>&1 < /dev/null & )
sleep 9
echo "  asr 节点: $(pgrep -f xf_asr_usb_node | head -1)"
AUD=$(tail -80 /tmp/asr.log 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | grep -c AudioWrite)
echo "  音频流 AudioWrite: $AUD"

# 3) 在线 TTS（播报"我在"）
if ! pgrep -f tts_net >/dev/null 2>&1; then
    ( setsid nohup python2 -u /home/ub/Hajimi/LXH/_tools/tts_net.py > /tmp/tts_asr.log 2>&1 < /dev/null & )
    sleep 2
fi
echo "  tts: $(pgrep -f tts_net | head -1)"
