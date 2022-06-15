/*
 * Copyright (C) 2022 Michael Ferguson
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

#include <robot_calibration/optimization/params.hpp>
#include <gtest/gtest.h>

TEST(OptimizationParamsTests, test_rosparam_loading)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("optimization_param_tests");
  robot_calibration::OptimizationParams params;
  params.LoadFromROS(node, "first_calibration_step");

  EXPECT_EQ(static_cast<size_t>(14), params.free_params.size());

  EXPECT_EQ(static_cast<size_t>(2), params.free_frames.size());
  EXPECT_EQ("head_camera_rgb_joint", params.free_frames[0].name);
  EXPECT_TRUE(params.free_frames[0].x);
  EXPECT_TRUE(params.free_frames[0].y);
  EXPECT_FALSE(params.free_frames[1].roll);

  EXPECT_EQ(static_cast<size_t>(2), params.models.size());
  EXPECT_EQ("arm", params.models[0].name);
  EXPECT_EQ("chain3d", params.models[0].type);
  EXPECT_EQ("gripper_led_frame", params.models[0].frame);
  EXPECT_EQ("camera", params.models[1].name);
  EXPECT_EQ("camera3d", params.models[1].type);
  EXPECT_EQ("head_camera_rgb_optical_frame", params.models[1].frame);

  EXPECT_EQ(static_cast<size_t>(2), params.error_blocks.size());
  // Check generic parameter values
  EXPECT_EQ("hand_eye", params.error_blocks[0]->name);
  EXPECT_EQ("chain3d_to_chain3d", params.error_blocks[0]->type);
  EXPECT_EQ("restrict_camera", params.error_blocks[1]->name);
  EXPECT_EQ("outrageous", params.error_blocks[1]->type);
  // Check specific values
  auto block0 = std::dynamic_pointer_cast<robot_calibration::OptimizationParams::Chain3dToChain3dParams>(params.error_blocks[0]);
  EXPECT_EQ("camera", block0->model_a);
  EXPECT_EQ("arm", block0->model_b);
  auto block1 = std::dynamic_pointer_cast<robot_calibration::OptimizationParams::OutrageousParams>(params.error_blocks[1]);
  EXPECT_EQ("head_camera_rgb_joint", block1->param);
  EXPECT_EQ(0.0, block1->joint_scale);
  EXPECT_EQ(0.1, block1->position_scale);
  EXPECT_EQ(0.1, block1->rotation_scale);

  EXPECT_EQ(static_cast<size_t>(1), params.free_frames_initial_values.size());
  EXPECT_EQ("checkerboard", params.free_frames_initial_values[0].name);
  EXPECT_EQ(0.0, params.free_frames_initial_values[0].x);
  EXPECT_EQ(1.0, params.free_frames_initial_values[0].y);
  EXPECT_EQ(2.0, params.free_frames_initial_values[0].z);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}