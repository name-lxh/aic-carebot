#!/bin/bash
# restart_tts_and_test.sh —— 重启在线 TTS（自动选播放声卡），播一句测试
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213
export PYTHONIOENCODING=utf-8

for p in $(pgrep -f tts_net 2>/dev/null); do
    [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null
done
sleep 1
setsid nohup python2 -u /home/ub/Hajimi/LXH/_tools/tts_net.py > /tmp/tts_new.log 2>&1 < /dev/null &
sleep 3
echo "=== tts 启动日志 ==="
grep -E "设备|device|就绪|ready" /tmp/tts_new.log 2>/dev/null | head -4

echo "=== 播一句测试 ==="
timeout 25 rostopic pub -1 /voice/say std_msgs/String "data: '测试，声音正常'" >/dev/null 2>&1
sleep 4
tail -4 /tmp/tts_new.log 2>/dev/null
