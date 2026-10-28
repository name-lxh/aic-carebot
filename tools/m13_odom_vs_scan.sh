#!/bin/bash
# ============================================================================
#  m13_odom_vs_scan.sh —— 判定 odom 与真实运动是否一致
# ============================================================================
#  背景：导航时目标在西北，机器人却往东南走。
#        要么 AMCL 定位错了，要么 odom 的方向和真实运动不一致。
#
#  做法（最干净的判据）：
#    1. 用 localize.py 做**全局匹配**得到当前真实位姿 A
#    2. 记录 odom_clean 的位置
#    3. 命令机器人**前进 0.20 m**
#    4. 再全局匹配得到位姿 B，再记录 odom
#    5. 比较两个位移向量：
#         · 方向一致、长度接近  -> odom 可信
#         · 方向相反            -> **odom 朝向符号错**（IMU yaw 约定问题）
#         · 长度差很多          -> 轮速标定问题
# ============================================================================
set -u

LXH=/home/ub/Hajimi/LXH
M=$LXH/catkin_ws/src/carebot_nav_kit/maps
export ROS_IP=192.168.8.213
export ROS_MASTER_URI=http://192.168.8.213:11399
source "$LXH/env.sh"
cd "$LXH/_tools" || exit 1

echo "════════ 1. 先停下所有运动相关节点 ════════"
pkill -x move_base 2>/dev/null
pkill -f '[m]9_nav_go' 2>/dev/null
sleep 2
for i in 1 2 3; do timeout 2 rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' >/dev/null 2>&1; done
echo "  已停"

odom_pos () {
    timeout 8 python -c "
from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('op', anonymous=True)
m = rospy.wait_for_message('/odom_clean', Odometry, timeout=8)
p = m.pose.pose.position
print('%.4f %.4f' % (p.x, p.y))
" 2>/dev/null
}

scan_pose () {
    timeout 480 python localize.py "$M/arena_clean.pgm" "$M/arena_clean.yaml" 2>/dev/null \
        | grep '建议初始位姿' | sed 's/.*x=\([-0-9.]*\) y=\([-0-9.]*\) yaw=\([-0-9.]*\).*/\1 \2 \3/'
}

echo ""
echo "════════ 2. 位姿 A ════════"
OA=$(odom_pos)
echo "  odom_clean: $OA"
echo "  全局匹配中（约 1~2 分钟）..."
SA=$(scan_pose)
echo "  scan 匹配: $SA"

if [ -z "$SA" ] || [ -z "$OA" ]; then
    echo "  [FAIL] 取不到位姿，中止"
    exit 1
fi

echo ""
echo "════════ 3. 命令前进 0.20 m ════════"
setsid nohup rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.08}}' > /dev/null 2>&1 < /dev/null &
PUB=$!
sleep 3.2
kill $PUB 2>/dev/null
pkill -x rostopic 2>/dev/null
for i in 1 2 3; do timeout 2 rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' >/dev/null 2>&1; done
sleep 2
echo "  已停"

echo ""
echo "════════ 4. 位姿 B ════════"
OB=$(odom_pos)
echo "  odom_clean: $OB"
echo "  全局匹配中..."
SB=$(scan_pose)
echo "  scan 匹配: $SB"

echo ""
echo "════════ 5. 比较位移 ════════"
python - "$OA" "$OB" "$SA" "$SB" <<'PYEOF' | sed 's/^/  /'
from __future__ import print_function
import math, sys
oa = [float(v) for v in sys.argv[1].split()]
ob = [float(v) for v in sys.argv[2].split()]
sa = [float(v) for v in sys.argv[3].split()]
sb = [float(v) for v in sys.argv[4].split()]

dox, doy = ob[0]-oa[0], ob[1]-oa[1]
dsx, dsy = sb[0]-sa[0], sb[1]-sa[1]
lo = math.hypot(dox, doy)
ls = math.hypot(dsx, dsy)

print('odom 位移: (%+.3f, %+.3f)  长度 %.3f m' % (dox, doy, lo))
print('scan 位移: (%+.3f, %+.3f)  长度 %.3f m' % (dsx, dsy, ls))

if lo < 0.02 or ls < 0.02:
    print('[?] 有一边几乎没动，数据不足（可能没发出去或匹配失败）')
    raise SystemExit(0)

ao = math.degrees(math.atan2(doy, dox))
as_ = math.degrees(math.atan2(dsy, dsx))
diff = (ao - as_ + 180) % 360 - 180
print('odom 方向 %.1f°   scan 方向 %.1f°   夹角 %+.1f°' % (ao, as_, diff))
print('长度比 %.2f' % (lo / ls if ls else 0))
print('')
if abs(diff) < 40:
    print('[OK] 方向一致 —— odom 与真实运动相符')
elif abs(abs(diff) - 180) < 50:
    print('[!!] 方向**相反** —— odom/IMU 的朝向符号有问题！')
else:
    print('[!!] 方向差异大（%.0f°）—— odom 与实际不符' % diff)
PYEOF

echo ""
echo "════════ 完成 ════════"
