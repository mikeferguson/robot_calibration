/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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

#include <robot_calibration/capture/chain_manager.h>
#include <gtest/gtest.h>

TEST(ChainManagerTests, test_rosparam_loading)
{
  ros::NodeHandle nh("~");
  robot_calibration::ChainManager manager(nh, 0.001);

  std::vector<std::string> chain_names = manager.getChains();
  EXPECT_EQ(2, chain_names.size());

  std::vector<std::string> joint_names = manager.getChainJointNames("arm");
  EXPECT_EQ(7, joint_names.size());
  EXPECT_EQ("arm_lift_joint", joint_names[0]);
  EXPECT_EQ("arm_shoulder_pan_joint", joint_names[1]);
  EXPECT_EQ("arm_shoulder_lift_joint", joint_names[2]);
  EXPECT_EQ("arm_upperarm_roll_joint", joint_names[3]);
  EXPECT_EQ("arm_elbow_flex_joint", joint_names[4]);
  EXPECT_EQ("arm_wrist_flex_joint", joint_names[5]);
  EXPECT_EQ("arm_wrist_roll_joint", joint_names[6]);

  joint_names = manager.getChainJointNames("not_a_chain");
  EXPECT_EQ(0, joint_names.size());

  joint_names = manager.getChainJointNames("head");
  EXPECT_EQ(2, joint_names.size());
  EXPECT_EQ("head_pan_joint", joint_names[0]);
  EXPECT_EQ("head_tilt_joint", joint_names[1]);

  std::string group_name = manager.getPlanningGroupName("arm");
  EXPECT_EQ("arm_group", group_name);

  group_name = manager.getPlanningGroupName("head");
  EXPECT_EQ("", group_name);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chain_manager_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}