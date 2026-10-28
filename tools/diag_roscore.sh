#!/bin/bash
# 诊断 roscore 为什么不起来 / 卡住
echo "════════ 1. 主机名与解析 ════════"
echo "  hostname:      $(hostname)"
echo "  hostname -f:   $(hostname -f 2>&1)"
echo "  /etc/hostname: $(cat /etc/hostname 2>/dev/null)"
echo "  /etc/hosts:"
cat /etc/hosts | sed 's/^/      /'

echo ""
echo "════════ 2. 主机名能不能解析 ════════"
for n in $(hostname) $(hostname -f 2>/dev/null) localhost ubuntu; do
    r=$(getent hosts "$n" 2>/dev/null)
    printf '  %-20s -> %s\n' "$n" "${r:-★解析不了}"
done

echo ""
echo "════════ 3. 端口 11311 占用 ════════"
ss -ltnp 2>/dev/null | grep 11311 | sed 's/^/  /' || echo "  · 没占用"

echo ""
echo "════════ 4. ★ 带 -v 启动 roscore，看它卡在哪（15 秒）════════"
set +u
source /opt/ros/kinetic/setup.bash
set -u
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"
echo "  ROS_HOSTNAME=${ROS_HOSTNAME:-（未设）}"
echo "  ROS_IP=${ROS_IP:-（未设）}"
echo "  --- 输出 ---"
timeout 15 roscore -v 2>&1 | head -25 | sed 's/^/      /'
echo "  --- 结束（超时=卡住，正常应该一直跑着）---"

echo ""
echo "════════ 5. 试试显式设 ROS_HOSTNAME/ROS_IP ════════"
echo "  --- 用 ROS_IP=192.168.43.233 ---"
ROS_IP=192.168.43.233 timeout 12 roscore 2>&1 | head -12 | sed 's/^/      /'

echo ""
echo "  --- 用 ROS_HOSTNAME=localhost ---"
ROS_HOSTNAME=localhost timeout 12 roscore 2>&1 | head -12 | sed 's/^/      /'

echo ""
echo "════════ 6. python 能否正常起 XML-RPC ════════"
timeout 10 python - <<'PYEOF' 2>&1 | sed 's/^/      /'
from __future__ import print_function
import socket
print('hostname = %s' % socket.gethostname())
try:
    print('gethostbyname(%s) = %s' % (socket.gethostname(), socket.gethostbyname(socket.gethostname())))
except Exception as e:
    print('★ gethostbyname 失败: %r' % e)
try:
    print('gethostbyname(ubuntu) = %s' % socket.gethostbyname('ubuntu'))
except Exception as e:
    print('★ gethostbyname(ubuntu) 失败: %r' % e)
PYEOF
