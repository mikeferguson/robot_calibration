#!/usr/bin/env python3

import os
import sys

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

def main(argv=sys.argv[1:]):
    config_file = os.path.join(os.getenv("CONFIG_DIR"), "error_block_tests2.yaml")

    test_node = Node(
        executable=[os.getenv("TEST_EXECUTABLE")],
        name="error_block_tests2",
        parameters=[config_file],
        output="screen",
    )

    ld = LaunchDescription()
    lts = LaunchTestService()
    lts.add_test_action(ld, test_node)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)

if __name__ == "__main__":
    sys.exit(main())
