# -*- coding: utf-8 -*-
"""m40_wait_ready.py —— 快速启动的就绪判定（并负责发初始位姿）

    python2 m40_wait_ready.py [--deadline 22] [--mission]

做四件事：
  1. 等关键话题有**数据**（不是只注册）：/scan、/scan_filtered、/odom_clean、/map、相机
  2. 发 /initialpose（起点固定 0,0,0）
  3. 等 map->base_footprint 的 TF 真正出现（AMCL 广播出来）
  4. 打一张表 + 总耗时
全通过返回 0，任一超时返回 2（调用方据此决定要不要硬跑）。
"""
from __future__ import print_function

import codecs
import sys
import time

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, LaserScan

for _n in ('stdout', 'stderr'):
    _s = getattr(sys, _n, None)
    _e = (getattr(_s, 'encoding', None) or '').lower()
    if _s is not None and _e not in ('utf-8', 'utf8'):
        try:
            setattr(sys, _n, codecs.getwriter('utf-8')(_s))
        except Exception:
            pass

TOPICS = [
    ('/scan', LaserScan),
    ('/scan_filtered', LaserScan),
    ('/odom_clean', Odometry),
    ('/map', OccupancyGrid),
    ('/camera/rgb/image_raw', Image),
]


def main():
    deadline = 22.0
    if '--deadline' in sys.argv:
        deadline = float(sys.argv[sys.argv.index('--deadline') + 1])

    rospy.init_node('wait_ready', anonymous=True)
    t_start = time.time()

    print(u'  ── 等数据 ──')
    ok = True
    for topic, msg_type in TOPICS:
        left = max(0.5, deadline - (time.time() - t_start))
        # 相机（OpenNI）冷启动要 8~12 s，单独给足时间，别用 6 s 一刀切
        cap = 12.0 if 'camera' in topic else 6.0
        try:
            rospy.wait_for_message(topic, msg_type, timeout=min(cap, left))
            print(u'    %-24s ✓' % topic)
        except Exception:
            print(u'    %-24s ✗ 超时' % topic)
            ok = False

    # 发初始位姿（起点固定 0,0,0）
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
    rospy.sleep(0.3)
    for _ in range(5):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                               0, 0.25, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0.0685]
        pub.publish(msg)
        rospy.sleep(0.25)
    print(u'  ── 已发 /initialpose (0,0,0) ──')

    print(u'  ── 等 map->base_footprint ──')
    listener = tf.TransformListener()
    got = False
    while time.time() - t_start < deadline and not rospy.is_shutdown():
        try:
            listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            got = True
            break
        except Exception:
            time.sleep(0.2)
    print(u'    TF map->base_footprint    %s' % (u'✓' if got else u'✗'))

    # 服务
    print(u'  ── 服务 ──')
    for name in ('/qr/detect_once', '/face/verify', '/iot/refresh'):
        left = max(0.5, deadline + 6.0 - (time.time() - t_start))
        try:
            rospy.wait_for_service(name, timeout=min(5.0, left))
            print(u'    %-24s ✓' % name)
        except Exception:
            print(u'    %-24s ✗' % name)
            ok = False

    # move_base 动作服务
    left = max(0.5, deadline + 6.0 - (time.time() - t_start))
    mb = False
    try:
        import actionlib
        from move_base_msgs.msg import MoveBaseAction
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        mb = ac.wait_for_server(rospy.Duration(min(4.0, left)))
    except Exception:
        mb = False
    print(u'    %-24s %s' % ('move_base action', u'✓' if mb else u'✗'))

    el = time.time() - t_start
    print(u'')
    print(u'  就绪耗时 %.1f s   %s' % (el, u'全绿' if (ok and got and mb) else u'有缺项'))
    return 0 if (ok and got and mb) else 2


if __name__ == '__main__':
    sys.exit(main())
