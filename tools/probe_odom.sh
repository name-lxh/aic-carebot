#!/bin/bash
# 量 /odom_clean：机器人静止时它应该**纹丝不动**
set +u
source /opt/ros/kinetic/setup.bash
source /home/ub/omniWheelCareRobot/rosCode/devel/setup.bash 2>/dev/null
set -u
export ROS_IP=192.168.43.233

echo "════════ 1. /odom_clean 有没有在发 ════════"
timeout 8 rostopic hz /odom_clean 2>&1 | head -3 | sed 's/^/  /'

echo ""
echo "════════ 2. ★ 静止 12 秒，看位置变不变 ════════"
timeout 30 python - <<'PYEOF' 2>&1 | sed 's/^/  /'
from __future__ import print_function
import math, time
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('odom_clean_probe', anonymous=True)
try:
    m0 = rospy.wait_for_message('/odom_clean', Odometry, timeout=10)
except Exception as e:
    print('读不到 /odom_clean: %r' % e)
    raise SystemExit(0)
def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
x0, y0, a0 = m0.pose.pose.position.x, m0.pose.pose.position.y, yaw(m0.pose.pose.orientation)
print('起点  x=%.4f y=%.4f yaw=%.2f' % (x0, y0, math.degrees(a0)))
t0 = time.time(); n = 0; mx = my = 0.0
while time.time() - t0 < 12.0 and not rospy.is_shutdown():
    try:
        m = rospy.wait_for_message('/odom_clean', Odometry, timeout=2)
    except Exception:
        continue
    n += 1
    dx = m.pose.pose.position.x - x0
    dy = m.pose.pose.position.y - y0
    mx = max(mx, abs(dx)); my = max(my, abs(dy))
print('样本 %d' % n)
print('最大位移 dx=%.4f  dy=%.4f  m' % (mx, my))
if mx < 0.01 and my < 0.01:
    print('✔ odom 静止不漂 —— gmapping 膨胀不是 odom 造成的')
else:
    print('★ odom 在漂，需要查为什么')
PYEOF

echo ""
echo "════════ 3. /cmd_vel 有没有人在发（除了我们）════════"
timeout 8 rostopic info /cmd_vel 2>&1 | sed -n '/Publishers/,/Subscribers/p' | sed 's/^/  /'
echo "  最近 3 条 /cmd_vel："
timeout 12 rostopic echo -n3 /cmd_vel 2>&1 | grep -E 'x:|z:' | head -8 | sed 's/^/      /'

echo ""
echo "════════ 4. gmapping 日志（看它怎么说）════════"
tail -20 /tmp/s2_gmapping.log | sed 's/^/  /'

echo ""
echo "════════ 5. odom_imu 日志 ════════"
tail -12 /tmp/s2_odomimu.log | sed 's/^/  /'

echo ""
echo "════════ 6. map 元信息随时间变吗 ════════"
for i in 1 2 3; do
    timeout 15 python - <<'PYEOF' 2>&1 | sed 's/^/  /'
from __future__ import print_function
import rospy
from nav_msgs.msg import OccupancyGrid
rospy.init_node('mi', anonymous=True)
m = rospy.wait_for_message('/map', OccupancyGrid, timeout=10)
print('%dx%d  origin (%.2f, %.2f)' % (m.info.width, m.info.height,
      m.info.origin.position.x, m.info.origin.position.y))
PYEOF
    sleep 4
done
