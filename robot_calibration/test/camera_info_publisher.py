#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__("fake_camera_info")
        self.publisher = self.create_publisher(CameraInfo, "/head_camera2/depth/camera_info", 5)

        # This is for testing the DepthCameraInfoManager
        self.declare_parameter("z_offset_mm", 0)
        self.declare_parameter("z_scaling", 1.0)

        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = CameraInfo()
        self.publisher.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    node = CameraInfoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
