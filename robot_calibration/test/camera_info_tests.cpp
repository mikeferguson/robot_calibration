/*
 * Copyright (C) 2018-2022 Michael Ferguson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <robot_calibration/util/camera_info.hpp>
#include <robot_calibration/util/depth_camera_info.hpp>
#include <gtest/gtest.h>

TEST(CameraInfoTests, test_update_camera_info)
{
  sensor_msgs::msg::CameraInfo start;
  start.p[0] = 1.0;  // FX
  start.p[2] = 2.0;  // CX
  start.p[5] = 3.0;  // FY
  start.p[6] = 4.0;  // CY

  start.k[0] = 5.0;  // FX
  start.k[2] = 6.0;  // CX
  start.k[4] = 7.0;  // FY
  start.k[5] = 8.0;  // CY

  sensor_msgs::msg::CameraInfo end =
  	robot_calibration::updateCameraInfo(0.0, 0.0, 0.0, 0.0, start);

  EXPECT_EQ(start.p[0], end.p[0]);
  EXPECT_EQ(start.p[2], end.p[2]);
  EXPECT_EQ(start.p[5], end.p[5]);
  EXPECT_EQ(start.p[6], end.p[6]);
  EXPECT_EQ(start.k[0], end.k[0]);
  EXPECT_EQ(start.k[2], end.k[2]);
  EXPECT_EQ(start.k[4], end.k[4]);
  EXPECT_EQ(start.k[5], end.k[5]);
}

TEST(CameraInfoTests, test_extended_camera_info)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("camera_info_tests");
  robot_calibration::DepthCameraInfoManager manager;

  rclcpp::Logger logger = node->get_logger();
  manager.init("test_manager", node, logger);

  robot_calibration_msgs::msg::ExtendedCameraInfo eci =
    manager.getDepthCameraInfo();

  ASSERT_EQ(static_cast<size_t>(2), eci.parameters.size());
  EXPECT_EQ(eci.parameters[0].name, "z_offset_mm");
  EXPECT_EQ(eci.parameters[0].value, 2.0);
  EXPECT_EQ(eci.parameters[1].name, "z_scaling");
  EXPECT_EQ(eci.parameters[1].value, 1.1);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}