#!/bin/bash
# read_sensors.sh —— 只起 master + IoT 桥接，读取全部传感器数据（不动车、不起导航）
set -u
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399
export ROS_IP=192.168.8.213

echo "════ 1. master ════"
if ! ss -ltn 2>/dev/null | grep -q ':11399'; then
    setsid nohup roscore -p 11399 > /tmp/rs_roscore.log 2>&1 < /dev/null &
    for i in $(seq 1 80); do ss -ltn 2>/dev/null | grep -q ':11399' && break; sleep 0.25; done
fi
ss -ltn 2>/dev/null | grep -q ':11399' && echo "  OK" || { echo "  FAIL"; exit 1; }

echo "════ 2. 重启 IoT 桥接 ════"
for p in $(pgrep -f "iot_bridge" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 2
setsid nohup roslaunch carebot_iot iot.launch \
    token_file:=/home/ub/Hajimi/LXH/.ha_token \
    > /tmp/rs_iot.log 2>&1 < /dev/null &
for i in $(seq 1 60); do
    timeout 4 rosservice list 2>/dev/null | grep -qx /iot/refresh && break
    sleep 0.5
done
if ! timeout 6 rosservice list 2>/dev/null | grep -qx /iot/refresh; then
    echo "  服务未就绪，日志："
    sed 's/\x1b\[[0-9;]*m//g' /tmp/rs_iot.log 2>/dev/null | tail -6
    exit 1
fi
echo "  /iot/refresh OK"

echo "════ 3. 读取传感器 ════"
timeout 30 rosservice call /iot/refresh 2>&1 | head -60
