# -*- coding: utf-8 -*-
"""make_plan_test.py —— 只算路径不动车，验证全局规划器能不能出路径

    python2 make_plan_test.py [x y] ...

不传参数就用任务里的关键航点。用 move_base 的 /make_plan 服务：
它**只做全局规划，不执行**，所以车不会动。
"""
from __future__ import print_function

import codecs
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path

# 注意：这里**不包装 stdout**。本脚本全部用 utf-8 字节串输出，
# 一旦套上 codecs.getwriter('utf-8')（它只接受 unicode），中文字节就会炸。

GOALS = [
    ('卧室门口', 1.77, 0.09),
    ('书房门口', 1.88, -1.93),
    ('厨房门口', 1.90, -0.95),
    ('客厅入口', 0.42, -0.90),
    ('扫描点', 0.70, -1.50),
    ('回起点', 0.00, 0.00),
]


def main():
    argv = [a for a in sys.argv[1:] if a != '--from']
    if '--from' in sys.argv:
        i = sys.argv.index('--from')
        argv = [a for a in argv if a not in (sys.argv[i + 1], sys.argv[i + 2])]
    goals = GOALS
    if len(argv) >= 2:
        goals = [('自定义(%.2f,%.2f)' % (float(argv[0]), float(argv[1])),
                  float(argv[0]), float(argv[1]))]

    rospy.init_node('make_plan_test', anonymous=True)
    rospy.wait_for_service('/move_base/make_plan', timeout=10)
    srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

    # --from x y : 指定规划起点（用来复现"在某个点卡死"）
    sx, sy = 0.0, 0.0
    if '--from' in sys.argv:
        i = sys.argv.index('--from')
        sx, sy = float(sys.argv[i + 1]), float(sys.argv[i + 2])
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = rospy.Time.now()
    start.pose.position.x = sx
    start.pose.position.y = sy
    start.pose.orientation.w = 1.0
    print('  规划起点 (%.2f, %.2f)' % (sx, sy))

    ok = 0
    for name, x, y in goals:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        try:
            # GetPlan 服务的调用签名是 (start, goal, tolerance) 三个参数
            resp = srv(start, goal, 0.25)
            n = len(resp.plan.poses) if resp.plan else 0
            if n > 0:
                ok += 1
                print('  %-10s (%.2f, %.2f)  OK plan %d poses' % (name, x, y, n))
            else:
                print('  %-10s (%.2f, %.2f)  NO-PATH' % (name, x, y))
        except Exception as exc:
            detail = str(exc)
            print('  %-10s (%.2f, %.2f)  FAIL: %s' % (name, x, y, detail[:90]))
    print(u'')
    print('  OK %d / %d' % (ok, len(goals)))
    return 0 if ok == len(goals) else 1


if __name__ == '__main__':
    sys.exit(main())
