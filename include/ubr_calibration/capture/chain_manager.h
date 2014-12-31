/*
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
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

#ifndef UBR_CALIBRATION_CAPTURE_CHAIN_MANAGER_H_
#define UBR_CALIBRATION_CAPTURE_CHAIN_MANAGER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

namespace ubr_calibration
{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

/**
 *  \brief Manages moving joints to a new pose, determining when they
 *  are settled, and returning current joint_states.
 */
class ChainManager
{
public:
  ChainManager(ros::NodeHandle & n) :
    head_client_("/head_controller/follow_joint_trajectory", true),
    arm_client_("/arm_controller/follow_joint_trajectory", true)
  {
    subscriber_ = n.subscribe("/joint_states", 1, &ChainManager::stateCallback, this);

    ROS_INFO("Waiting for head_controller/follow_joint_trajectory...");
    head_client_.waitForServer();

    ROS_INFO("Waiting for arm_controller/follow_joint_trajectory...");
    arm_client_.waitForServer();

    head_joints_.push_back("head_pan_joint");
    head_joints_.push_back("head_tilt_joint");

    arm_joints_.push_back("shoulder_pan_joint");
    arm_joints_.push_back("shoulder_lift_joint");
    arm_joints_.push_back("upperarm_roll_joint");
    arm_joints_.push_back("elbow_flex_joint");
    arm_joints_.push_back("forearm_roll_joint");
    arm_joints_.push_back("wrist_flex_joint");
    arm_joints_.push_back("wrist_roll_joint");
  }

  /**
   *  \brief Send commands to all managed joints. The ChainManager automatically figures out
   *         which controller to send these to.
   *  \returns False if failed.
   */
  bool moveToState(const sensor_msgs::JointState& state);

  /**
   *  \brief Wait for joints to settle.
   */
  bool waitToSettle();

  /**
   *  \brief Get the current JointState message.
   */
  bool getState(sensor_msgs::JointState* state);

private:
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  trajectory_msgs::JointTrajectoryPoint makePoint(const sensor_msgs::JointState& state,
                                                  const std::vector<std::string> joints);

  ros::Subscriber subscriber_;
  sensor_msgs::JointState state_;

  TrajectoryClient head_client_;
  TrajectoryClient arm_client_;

  std::vector<std::string> head_joints_;
  std::vector<std::string> arm_joints_;
};

}  // namespace ubr_calibration

#endif  // UBR_CALIBRATION_CAPTURE_CHAIN_MANAGER_H_
