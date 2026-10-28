#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image


def main():
    rospy.init_node('snap_camera_frame')
    bridge = cv_bridge.CvBridge()

    def cb(msg):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logerr('convert failed: %s', exc)
            rospy.signal_shutdown('failed')
            return
        cv2.imwrite('/tmp/camera_frame.jpg', img)
        rospy.loginfo('saved /tmp/camera_frame.jpg %s', img.shape)
        rospy.signal_shutdown('saved')

    rospy.Subscriber('/camera/rgb/image_raw', Image, cb, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
