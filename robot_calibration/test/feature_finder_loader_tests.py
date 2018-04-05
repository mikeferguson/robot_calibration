#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

if __name__ == "__main__":

    rospy.init_node("fake_camera_info")
    pub = rospy.Publisher("/head_camera/depth/camera_info", CameraInfo, queue_size=5)
    msg = CameraInfo()

    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.sleep(0.1)
