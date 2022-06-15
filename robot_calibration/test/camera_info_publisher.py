#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__("fake_camera_info")
        self.publisher = self.create_publisher(CameraInfo, "/head_camera/depth/camera_info", 5)


if __name__ == "__main__":
    rclpy.init()
    node = CameraInfoPublisher()

    try:
        while rclpy.ok():
            msg = CameraInfo()
            node.publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
