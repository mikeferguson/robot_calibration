/*
 * Copyright (C) 2015 Fetch Robotics Inc.
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

#include <urdf/model.h>
#include <robot_calibration/ceres/optimizer.h>
#include <gtest/gtest.h>

std::string robot_description =
"<?xml version='1.0' ?>"
"<robot name='maxwell'>"
"  <link name='base_link'/>"
"  <joint name='torso_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='-0.00635 0 0.7914'/>"
"    <parent link='base_link'/>"
"    <child link='torso_link'/>"
"  </joint>"
"  <link name='torso_link'/>"
"  <joint name='torso_actuator_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='torso_link'/>"
"    <child link='torso_actuator_link'/>"
"  </joint>"
"  <link name='torso_actuator_link'/>"
"  <joint name='arm_lift_joint' type='prismatic'>"
"    <axis xyz='0 0 1'/>"
"    <limit effort='30' lower='-0.464' upper='0' velocity='0.0508'/>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='torso_link'/>"
"    <child link='arm_lift_link'/>"
"  </joint>"
"  <link name='arm_lift_link'/>"
"  <joint name='arm_base_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.0611 0 0'/>"
"    <parent link='arm_lift_link'/>"
"    <child link='arm_link'/>"
"  </joint>"
"  <link name='arm_link'/>"
"  <joint name='arm_shoulder_pan_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='arm_link'/>"
"    <child link='arm_shoulder_pan_servo_link'/>"
"  </joint>"
"  <link name='arm_shoulder_pan_servo_link'/>"
"  <joint name='arm_shoulder_pan_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0.0235'/>"
"    <axis xyz='0 0 1'/>"
"    <limit effort='30' lower='-1.57' upper='1.57' velocity='0.524'/>"
"    <parent link='arm_shoulder_pan_servo_link'/>"
"    <child link='arm_shoulder_pan_link'/>"
"  </joint>"
"  <link name='arm_shoulder_pan_link'/>"
"  <joint name='arm_shoulder_lift_servo_joint' type='fixed'>"
"    <origin rpy='0 -1.57 0' xyz='0 0 0.0526'/>"
"    <parent link='arm_shoulder_pan_link'/>"
"    <child link='arm_shoulder_lift_servo_link'/>"
"  </joint>"
"  <link name='arm_shoulder_lift_servo_link'/>"
"  <joint name='arm_shoulder_lift_joint' type='revolute'>"
"    <origin rpy='0 1.57 0' xyz='0 0 0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.77' upper='1.317' velocity='0.524'/>"
"    <parent link='arm_shoulder_lift_servo_link'/>"
"    <child link='arm_shoulder_lift_link'/>"
"  </joint>"
"  <link name='arm_shoulder_lift_link'/>"
"  <joint name='arm_upperarm_roll_servo_joint' type='fixed'>"
"    <origin rpy='1.57 1.57 0' xyz='0.0712978 0 0'/>"
"    <parent link='arm_shoulder_lift_link'/>"
"    <child link='arm_upperarm_roll_servo_link'/>"
"  </joint>"
"  <link name='arm_upperarm_roll_servo_link'/>"
"  <joint name='arm_upperarm_roll_joint' type='revolute'>"
"    <origin rpy='-1.57 0 1.57' xyz='0 0 0'/>"
"    <axis xyz='1 0 0'/>"
"    <limit effort='30' lower='-2' upper='2' velocity='0.524'/>"
"    <parent link='arm_upperarm_roll_servo_link'/>"
"    <child link='arm_upperarm_roll_link'/>"
"  </joint>"
"  <link name='arm_upperarm_roll_link'/>"
"  <joint name='arm_elbow_flex_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0.0869955 0 0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.57' upper='2.617' velocity='0.524'/>"
"    <parent link='arm_upperarm_roll_link'/>"
"    <child link='arm_elbow_flex_link'/>"
"  </joint>"
"  <link name='arm_elbow_flex_link'/>"
"  <joint name='arm_forearm_fixed_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='arm_elbow_flex_link'/>"
"    <child link='arm_forearm_link'/>"
"  </joint>"
"  <link name='arm_forearm_link'/>"
"  <joint name='arm_wrist_flex_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.125 0 0'/>"
"    <parent link='arm_forearm_link'/>"
"    <child link='arm_wrist_flex_servo_link'/>"
"  </joint>"
"  <link name='arm_wrist_flex_servo_link'/>"
"  <joint name='arm_wrist_flex_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0.0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.57' upper='1.57' velocity='0.785'/>"
"    <parent link='arm_wrist_flex_servo_link'/>"
"    <child link='arm_wrist_flex_link'/>"
"  </joint>"
"  <link name='arm_wrist_flex_link'/>"
"  <joint name='arm_wrist_roll_joint' type='revolute'>"
"    <axis xyz='1 0 0'/>"
"    <limit effort='30' lower='-2.617' upper='2.617' velocity='0.785'/>"
"    <origin rpy='0 0 0' xyz='0.031 0 0'/>"
"    <parent link='arm_wrist_flex_link'/>"
"    <child link='arm_wrist_roll_link'/>"
"  </joint>"
"  <link name='arm_wrist_roll_link'/>"
"  <joint name='gripper_joint' type='fixed'>"
"    <axis xyz='0 0 1'/>"
"    <origin rpy='0 0 0' xyz='0.15 0 -0.015'/>"
"    <parent link='arm_wrist_roll_link'/>"
"    <child link='gripper_led_frame'/>"
"  </joint>"
"  <link name='gripper_led_frame'/>"
"  <joint name='head_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0.512375'/>"
"    <parent link='torso_link'/>"
"    <child link='head_link'/>"
"  </joint>"
"  <link name='head_link'/>"
"  <joint name='head_pan_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.010 0 0.0254'/>"
"    <parent link='head_link'/>"
"    <child link='head_pan_servo_link'/>"
"  </joint>"
"  <link name='head_pan_servo_link'/>"
"  <joint name='head_pan_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0.019'/>"
"    <axis xyz='0 0 1'/>"
"    <limit effort='30' lower='-2.617' upper='2.617' velocity='1.0'/>"
"    <parent link='head_pan_servo_link'/>"
"    <child link='head_pan_link'/>"
"  </joint>"
"  <link name='head_pan_link'/>"
"  <joint name='head_tilt_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0.0415'/>"
"    <parent link='head_pan_link'/>"
"    <child link='head_tilt_servo_link'/>"
"  </joint>"
"  <link name='head_tilt_servo_link'/>"
"  <joint name='head_tilt_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.57' upper='1.57' velocity='1.0'/>"
"    <parent link='head_tilt_servo_link'/>"
"    <child link='head_tilt_link'/>"
"  </joint>"
"  <link name='head_tilt_link'/>"
"  <joint name='head_camera_frame_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0.026'/>"
"    <parent link='head_tilt_link'/>"
"    <child link='head_camera_frame'/>"
"  </joint>"
"  <link name='head_camera_frame'/>"
"  <joint name='head_camera_ir_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.01905 -0.0269875 0.032075'/>"
"    <parent link='head_camera_frame'/>"
"    <child link='head_camera_ir_link'/>"
"  </joint>"
"  <link name='head_camera_ir_link'/>"
"  <joint name='head_camera_ir_optical_frame_joint' type='fixed'>"
"    <origin rpy='-1.57 0.0 -1.57' xyz='0 0 0'/>"
"    <parent link='head_camera_ir_link'/>"
"    <child link='head_camera_ir_optical_frame'/>"
"  </joint>"
"  <link name='head_camera_ir_optical_frame'/>"
"  <joint name='head_camera_rgb_joint' type='fixed'>"
"    <origin rpy='0 -0.08 0' xyz='0 0.0552875 0'/>"
"    <parent link='head_camera_ir_link'/>"
"    <child link='head_camera_rgb_link'/>"
"  </joint>"
"  <link name='head_camera_rgb_link'/>"
"  <joint name='head_camera_rgb_optical_frame_joint' type='fixed'>"
"    <origin rpy='-1.57 0.0 -1.57' xyz='0 0 0'/>"
"    <parent link='head_camera_rgb_link'/>"
"    <child link='head_camera_rgb_optical_frame'/>"
"  </joint>"
"  <link name='head_camera_rgb_optical_frame'/>"
"</robot>";

TEST(ErrorBlockTests, error_blocks_maxwell)
{
  ros::NodeHandle nh("~");

  robot_calibration::Optimizer opt(robot_description);

  std::vector<robot_calibration_msgs::CalibrationData> data;
  robot_calibration_msgs::CalibrationData msg;

  // Match expected output from chain manager
  msg.joint_states.name.resize(10);
  msg.joint_states.name[0] = "arm_lift_flex_joint";
  msg.joint_states.name[1] = "arm_shoulder_pan_joint";
  msg.joint_states.name[2] = "arm_shoulder_lift_joint";
  msg.joint_states.name[3] = "arm_upperarm_roll_joint";
  msg.joint_states.name[4] = "arm_elbow_flex_joint";
  msg.joint_states.name[5] = "arm_wrist_flex_joint";
  msg.joint_states.name[6] = "arm_wrist_roll_joint";
  msg.joint_states.name[7] = "head_pan_joint";
  msg.joint_states.name[8] = "head_tilt_joint";
  msg.joint_states.name[9] = "arm_lift_joint";
  msg.joint_states.position.resize(10);
  msg.joint_states.position[0] = 0.0;
  msg.joint_states.position[1] = -0.814830;
  msg.joint_states.position[2] = -0.00022290000000002586;
  msg.joint_states.position[3] = 0.0;
  msg.joint_states.position[4] = -0.7087341;
  msg.joint_states.position[5] = 0.0;
  msg.joint_states.position[6] = 0.0;
  msg.joint_states.position[7] = -0.8280187999999999;
  msg.joint_states.position[8] = 0.6358500000000002;
  msg.joint_states.position[9] = 0.0;

  // Expectect output from led finder
  msg.observations.resize(2);
  msg.observations[0].sensor_name = "camera";
  msg.observations[1].sensor_name = "arm";

  msg.observations[0].features.resize(1);
  msg.observations[0].features[0].header.frame_id = "head_camera_rgb_optical_frame";
  msg.observations[0].features[0].point.x = -0.0143163670728;
  msg.observations[0].features[0].point.y = 0.111304592065;
  msg.observations[0].features[0].point.z = 0.522079317365;

  msg.observations[0].ext_camera_info.camera_info.P[0] = 100.0;  // fx
  msg.observations[0].ext_camera_info.camera_info.P[5] = 100.0;  // fy
  msg.observations[0].ext_camera_info.camera_info.P[2] = 320.0;  // cx
  msg.observations[0].ext_camera_info.camera_info.P[6] = 240.0;  // cy
  msg.observations[0].ext_camera_info.parameters.resize(2);
  msg.observations[0].ext_camera_info.parameters[0].name = "z_offset";
  msg.observations[0].ext_camera_info.parameters[0].value = 0.0;
  msg.observations[0].ext_camera_info.parameters[1].name = "z_scaling";
  msg.observations[0].ext_camera_info.parameters[1].value = 1.0;

  msg.observations[1].features.resize(1);
  msg.observations[1].features[0].header.frame_id = "gripper_led_frame";
  msg.observations[1].features[0].point.x = 0.0;
  msg.observations[1].features[0].point.y = 0.0;
  msg.observations[1].features[0].point.z = 0.0;

  // Add first data point
  data.push_back(msg);

  // Add a second data point that is just a little different
  msg.joint_states.position[1] = -0.019781999999999966;
  msg.joint_states.position[7] = 0.0;
  msg.observations[0].features[0].point.x = 0.0365330705881;
  msg.observations[0].features[0].point.y = 0.102609552493;
  msg.observations[0].features[0].point.z = 0.536061220027;
  data.push_back(msg);

  // And a third data point
  msg.joint_states.position[1] = 0.883596;
  msg.joint_states.position[7] = 0.9442135999999999;
  msg.observations[0].features[0].point.x = 0.0942445346646;
  msg.observations[0].features[0].point.y = 0.11409172323;
  msg.observations[0].features[0].point.z = 0.517497963716;
  data.push_back(msg);

  // Setup params
  robot_calibration::OptimizationParams params;
  params.LoadFromROS(nh);

  // Optimize
  opt.optimize(params, data, false);
  EXPECT_LT(opt.summary()->initial_cost, 1e-20);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "error_block_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
