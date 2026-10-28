#!/bin/bash
# 后台跑 V5 全流程（日志落盘，SSH 断了也不影响）
source /home/ub/Hajimi/LXH/env.sh
export ROS_MASTER_URI=http://192.168.8.213:11311
export ROS_IP=192.168.8.213
export PYTHONIOENCODING=utf-8
cd /home/ub/Hajimi/LXH/_tools || exit 1

for p in $(pgrep -f "m36_mission"); do kill "$p" 2>/dev/null; done
sleep 1
rm -f /tmp/m36_run7.log
( setsid nohup python2 -u m36_mission_v5.py > /tmp/m36_run7.log 2>&1 < /dev/null & )
sleep 3
pid=$(pgrep -f "m36_mission" | head -1)
echo "started pid=$pid"
tail -3 /tmp/m36_run7.log 2>/dev/null
