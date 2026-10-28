#!/bin/bash
# 起业务节点：相机 + 二维码 + 人脸 + IOT + 在线 TTS（车重启后需要重跑一次）
source /home/ub/Hajimi/LXH/env.sh
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
TOOLS=/home/ub/Hajimi/LXH/_tools

# 0) 清理旧实例
for pat in 'face_gate.py' 'qr_detector.py' 'iot_bridge.py' 'tts_net'; do
    for p in $(pgrep -f "$pat"); do kill "$p" 2>/dev/null; done
done
for p in $(pgrep -f 'astra'); do kill "$p" 2>/dev/null; done
sleep 2

# 1) 相机（Astra 走 OpenNI，不是 /dev/video）
# ★ 必须在**子 shell** 里 source 厂商的 devel：它会覆盖 ROS_PACKAGE_PATH，
#   在父 shell 里 source 会让后面的 carebot_qr/face/iot 全都找不到包。
( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
  setsid nohup roslaunch astra_launch astra.launch > /tmp/svc_astra.log 2>&1 < /dev/null & )
sleep 10
source /home/ub/Hajimi/LXH/env.sh

# 2) 二维码
setsid nohup roslaunch carebot_qr qr_detect.launch > /tmp/svc_qr.log 2>&1 < /dev/null &
# 3) 人脸
setsid nohup roslaunch carebot_face face.launch > /tmp/svc_face.log 2>&1 < /dev/null &
# 4) IOT（带令牌）
setsid nohup roslaunch carebot_iot iot.launch token_file:=/home/ub/Hajimi/LXH/.ha_token \
    > /tmp/svc_iot.log 2>&1 < /dev/null &
sleep 10

# 5) 在线 TTS
( setsid nohup python2 -u "$TOOLS/tts_net.py" > /tmp/svc_tts.log 2>&1 < /dev/null & )
sleep 3

echo "=== 相机 ==="
timeout 8 rostopic hz /camera/rgb/image_raw 2>&1 | grep -m1 -E "average|no new"
echo "=== 服务 ==="
for s in /qr/detect_once /face/verify /iot/refresh /iot/control; do
    printf "  %-20s " "$s"
    timeout 6 rosservice list 2>/dev/null | grep -qx "$s" && echo OK || echo "缺"
done
echo "=== 播报节点 ==="
echo "  tts_net: $(pgrep -f 'tts_net' | head -1)"
