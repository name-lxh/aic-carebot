# -*- coding: utf-8 -*-
"""map_free_x.py —— 查每个房间行上自由空间的 x 范围（确认房间有多深）"""
from __future__ import print_function
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node('map_free_x', anonymous=True)
g = rospy.wait_for_message('/map', OccupancyGrid, timeout=12)
info = g.info
res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
print(u'  地图 %dx%d  %.3f m/px  原点 (%.2f, %.2f)' % (info.width, info.height, res, ox, oy))
for yy in (0.0, -1.0, -1.05, -2.0, -2.05):
    row = int(round((yy - oy) / res))
    if row < 0 or row >= info.height:
        continue
    xs = [xx for xx in range(info.width)
          if g.data[row * info.width + xx] == 0 and ox + xx * res > 1.4]
    if xs:
        print(u'  y=%.2f  自由 x: %.2f ~ %.2f  (最深 %.2f)'
              % (yy, ox + min(xs) * res, ox + max(xs) * res, ox + max(xs) * res))
    else:
        print(u'  y=%.2f  该行 x>1.4 无自由格' % yy)
