#!/bin/bash
# setup_capture.sh —— 采样前的准备：目录、音量、master、相机、TTS
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
LXH=/home/ub/Hajimi/LXH

# 1) 我们自己的数据集/模型目录（跟着代码走，可迁移）
mkdir -p "$LXH/face_dataset/id2_raw" "$LXH/face_dataset/model"
SRC=/home/ub/omniWheelCareRobot/rosCode/src/face_recognition/script/source/trainer/trainer.yml
if [ -f "$SRC" ] && [ ! -f "$LXH/face_dataset/model/trainer.yml" ]; then
    cp "$SRC" "$LXH/face_dataset/model/trainer.yml"
    echo "  trainer 已复制到我们自己的目录"
fi

# 2) 音量降到现在的 40%（45% -> 18%）
amixer -c 0 sset PCM 18% >/dev/null 2>&1
VOL=$(amixer -c 0 get PCM 2>/dev/null | grep -o "\[[0-9]*%\]" | head -1)
echo "  音量: ${VOL:-未知}"

# 3) master
if ! pgrep -x rosmaster >/dev/null 2>&1; then
    setsid nohup roscore -p 11399 > /tmp/cap_roscore.log 2>&1 < /dev/null &
    sleep 7
fi

# 4) 相机（子 shell 里 source 厂商环境，避免覆盖我们的 ROS_PACKAGE_PATH）
if ! pgrep -f "camera_nodelet_manager" >/dev/null 2>&1; then
    ( source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
      setsid nohup roslaunch astra_launch astra.launch > /tmp/cap_astra.log 2>&1 < /dev/null & )
fi

# 5) 在线 TTS（播报"开始检测"）
if ! pgrep -f "tts_net" >/dev/null 2>&1; then
    ( setsid nohup python2 -u "$LXH/_tools/tts_net.py" > /tmp/cap_tts.log 2>&1 < /dev/null & )
fi

sleep 11
HZ=$(timeout 8 rostopic hz /camera/rgb/image_raw 2>&1 | grep -m1 -E "average|no new")
echo "  相机: ${HZ:-无数据}"
echo "  tts:  $(pgrep -f tts_net | head -1)"
echo "  目录: $LXH/face_dataset"
ls -la "$LXH/face_dataset" 2>/dev/null | head -4
