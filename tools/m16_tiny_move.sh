#!/bin/bash
# ============================================================================
#  m16_tiny_move.sh —— 极小步位移测试（测"命令->运动->odom"方向是否正确）
# ============================================================================
#  背景：之前用 move_base 导航，目标在西北、机器人却往东南跑。
#        已确认 odom 与真实运动一致（用全局匹配验证过），
#        所以问题在定位/地图。但为了排除"执行链路方向反"这种低级错误，
#        先用**最小动作**验证一次：
#
#      1. 设初始位姿（全局匹配给出）
#      2. 发 0.06 m/s 前进 **2 秒** = 约 0.12 m
#      3. 比较 odom 位移的方向与"车头朝向"是否一致
#
#  0.12 m 的位移在任何情况下都不足以撞墙（前向净空会有记录）。
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"

IX=${1:--0.520}
IY=${2:-1.140}
IYAW=${3:-195.0}
# 注意：localize 给的是 (0.520, -1.140, 195)，这里用参数传，默认按负数写法避免 shell 吃掉负号
IX=$(python -c "print(abs($IX))")
IY=$(python -c "print(-abs($IY))")

echo "════════ 0. 初始位姿 ════════"
echo "  将设为: x=$IX  y=$IY  yaw=$IYAW"

echo ""
echo "════════ 1. 发布 /initialpose ════════"
timeout 30 python - "$IX" "$IY" "$IYAW" <<'PYEOF' 2>&1 | sed 's/^/  /'
from __future__ import print_function
import math, sys, time
import rospy, tf
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rospy.init_node('setpose', anonymous=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
time.sleep(1.0)
p = PoseWithCovarianceStamped()
p.header.frame_id = 'map'
p.header.stamp = rospy.Time.now()
p.pose.pose.position.x = x
p.pose.pose.position.y = y
q = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw))
p.pose.pose.orientation.x = q[0]
p.pose.pose.orientation.y = q[1]
p.pose.pose.orientation.z = q[2]
p.pose.pose.orientation.w = q[3]
p.pose.covariance = [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                     0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]
for _ in range(6):
    pub.publish(p)
    time.sleep(0.3)
print('已发布')
time.sleep(3.0)
try:
    ap = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=10)
    pp = ap.pose.pose.position
    print('AMCL 收敛到: (%.3f, %.3f)' % (pp.x, pp.y))
except Exception as e:
    print('拿不到 /amcl_pose: %r' % e)
PYEOF

echo ""
echo "════════ 2. 记录运动前状态 ════════"
timeout 40 python - <<'PYEOF' 2>&1 | sed 's/^/  /'
from __future__ import print_function
import math, time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
rospy.init_node('pre', anonymous=True)
o = rospy.wait_for_message('/odom_clean', Odometry, timeout=10)
p = o.pose.pose.position
q = o.pose.pose.orientation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
print('odom   x=%.4f y=%.4f yaw=%.1f°' % (p.x, p.y, math.degrees(yaw)))
s = rospy.wait_for_message('/scan_filtered', LaserScan, timeout=10)
fin=[v for v in s.ranges if v==v and v!=float('inf') and v>0.02]
front=[s.ranges[i] for i in range(len(s.ranges))
       if s.ranges[i]==s.ranges[i] and s.ranges[i]!=float('inf')
       and abs(s.angle_min+i*s.angle_increment)<math.radians(35)]
print('前方(±35°)最近: %.2f m   全向最近: %.2f m'
      % (min(front) if front else -1, min(fin) if fin else -1))
with open('/tmp/pre_odom.txt','w') as f:
    f.write('%.6f %.6f %.6f\n' % (p.x, p.y, yaw))
PYEOF

echo ""
echo "════════ 3. 前进 0.06 m/s × 2 s（约 0.12 m）════════"
setsid nohup rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.06}}' > /dev/null 2>&1 < /dev/null &
sleep 2.0
pkill -x rostopic 2>/dev/null
for i in 1 2 3 4; do
    timeout 2 rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' >/dev/null 2>&1
    sleep 0.2
done
sleep 2

echo ""
echo "════════ 4. 记录运动后状态并比较 ════════"
timeout 40 python - <<'PYEOF' 2>&1 | sed 's/^/  /'
from __future__ import print_function
import math
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('post', anonymous=True)
o = rospy.wait_for_message('/odom_clean', Odometry, timeout=10)
p = o.pose.pose.position
q = o.pose.pose.orientation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
x0, y0, a0 = [float(v) for v in open('/tmp/pre_odom.txt').read().split()]
dx, dy = p.x - x0, p.y - y0
d = math.hypot(dx, dy)
print('位移: (%+.4f, %+.4f)  长度 %.4f m' % (dx, dy, d))
if d > 0.01:
    moved = math.degrees(math.atan2(dy, dx))
    head = math.degrees(a0)
    diff = (moved - head + 180) % 360 - 180
    print('位移方向 %.1f°   车头朝向 %.1f°   夹角 %+.1f°' % (moved, head, diff))
    print('')
    if abs(diff) < 40:
        print('[OK] 机器人朝**车头方向**前进 —— 执行链路方向正确')
    elif abs(abs(diff) - 180) < 50:
        print('[!!] 机器人朝**反方向**走 —— 底盘 cmd_vel 的 x 符号反了')
    else:
        print('[??] 方向偏差 %.0f°，需要进一步查' % diff)
else:
    print('[?] 几乎没动（%.4f m）—— 检查底盘/驱动是否响应 /cmd_vel' % d)
PYEOF

echo ""
echo "════════ 完成（机器人应已停）════════"
