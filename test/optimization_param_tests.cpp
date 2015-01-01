/*
 * Copyright (C) 2014 Fetch Robotics Inc.
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

#include <robot_calibration/ceres/optimization_params.h>
#include <gtest/gtest.h>

TEST(ChainManagerTests, test_rosparam_loading)
{
  ros::NodeHandle nh("~");
  robot_calibration::OptimizationParams params;
  params.LoadFromROS(nh);

  EXPECT_EQ(14, params.free_params.size());

  EXPECT_EQ(2, params.free_frames.size());
  EXPECT_EQ("head_camera_rgb_joint", params.free_frames[0].name);
  EXPECT_TRUE(params.free_frames[0].x);
  EXPECT_TRUE(params.free_frames[0].y);
  EXPECT_FALSE(params.free_frames[1].roll);

  EXPECT_EQ(2, params.models.size());
  EXPECT_EQ("arm", params.models[0].name);
  EXPECT_EQ("chain", params.models[0].type);
  EXPECT_EQ("gripper_led_frame", static_cast<std::string>(params.models[0].params["frame"]));

  EXPECT_EQ(2, params.error_blocks.size());
  EXPECT_EQ("hand_eye", params.error_blocks[0].name);
  EXPECT_EQ("camera3d_to_arm", params.error_blocks[0].type);
  EXPECT_EQ("arm", static_cast<std::string>(params.error_blocks[0].params["arm"]));
  EXPECT_EQ("camera", static_cast<std::string>(params.error_blocks[0].params["camera"]));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "optimization_param_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}