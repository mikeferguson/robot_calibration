#!/usr/bin/env python3

import os
import sys

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

def main(argv=sys.argv[1:]):
    test_node = Node(
        executable=[os.getenv("TEST_EXECUTABLE")],
        name="camera_info_tests",
        parameters=[{"test_manager.camera_driver": "/camera_info_publisher"}],
        output="screen",
    )

    info_publisher = Node(
        executable=[os.getenv("INFO_PUBLISHER")],
        name="camera_info_publisher",
        parameters=[{"z_offset_mm": 2,
                     "z_scaling": 1.1}],
        output="screen",
    )

    ld = LaunchDescription([info_publisher])
    lts = LaunchTestService()
    lts.add_test_action(ld, test_node)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)

if __name__ == "__main__":
    sys.exit(main())
