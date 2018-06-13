/*
 * Copyright (C) 2018 Michael Ferguson
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

#include <robot_calibration/camera_info.h>
#include <robot_calibration/capture/depth_camera.h>
#include <gtest/gtest.h>

TEST(CameraInfoTests, test_update_camera_info)
{
  sensor_msgs::CameraInfo start;
  start.P[0] = 1.0;  // FX
  start.P[2] = 2.0;  // CX
  start.P[5] = 3.0;  // FY
  start.P[6] = 4.0;  // CY

  start.K[0] = 5.0;  // FX
  start.K[2] = 6.0;  // CX
  start.K[4] = 7.0;  // FY
  start.K[5] = 8.0;  // CY

  sensor_msgs::CameraInfo end =
  	robot_calibration::updateCameraInfo(0.0, 0.0, 0.0, 0.0, start);

  EXPECT_EQ(start.P[0], end.P[0]);
  EXPECT_EQ(start.P[2], end.P[2]);
  EXPECT_EQ(start.P[5], end.P[5]);
  EXPECT_EQ(start.P[6], end.P[6]);
  EXPECT_EQ(start.K[0], end.K[0]);
  EXPECT_EQ(start.K[2], end.K[2]);
  EXPECT_EQ(start.K[4], end.K[4]);
  EXPECT_EQ(start.K[5], end.K[5]);
}

TEST(CameraInfoTests, test_extended_camera_info)
{
  ros::NodeHandle nh("~");
  robot_calibration::DepthCameraInfoManager manager;

  manager.init(nh);

  robot_calibration_msgs::ExtendedCameraInfo eci =
    manager.getDepthCameraInfo();

  ASSERT_EQ(static_cast<size_t>(2), eci.parameters.size());
  EXPECT_EQ(eci.parameters[0].name, "z_offset_mm");
  EXPECT_EQ(eci.parameters[0].value, 2.0);
  EXPECT_EQ(eci.parameters[1].name, "z_scaling");
  EXPECT_EQ(eci.parameters[1].value, 1.1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_info_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}