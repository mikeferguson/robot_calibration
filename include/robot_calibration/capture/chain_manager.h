/*
 * Copyright (C) 2014 Fetch Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_CAPTURE_CHAIN_MANAGER_H
#define ROBOT_CALIBRATION_CAPTURE_CHAIN_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

namespace robot_calibration
{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

/**
 *  @brief Manages moving joints to a new pose, determining when they
 *         are settled, and returning current joint_states.
 */
class ChainManager
{
  struct ChainController
  {
    ChainController(const std::string& topic) :
      client(topic, true)
    {
    }

    TrajectoryClient client;
    std::vector<std::string> joint_names;
  };

public:
  /**
   * @brief Constructor, sets up chains from ros parameters.
   * @param nh The node handle, sets namespace for parameters.
   */
  ChainManager(ros::NodeHandle& nh);

  /**
   *  @brief Send commands to all managed joints. The ChainManager automatically figures out
   *         which controller to send these to.
   *  @returns False if failed.
   */
  bool moveToState(const sensor_msgs::JointState& state);

  /**
   *  @brief Wait for joints to settle.
   */
  bool waitToSettle();

  /**
   *  @brief Get the current JointState message.
   */
  bool getState(sensor_msgs::JointState* state);

private:
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  trajectory_msgs::JointTrajectoryPoint makePoint(const sensor_msgs::JointState& state,
                                                  const std::vector<std::string>& joints);

  ros::Subscriber subscriber_;
  sensor_msgs::JointState state_;
  std::vector<boost::shared_ptr<ChainController> > controllers_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHAIN_MANAGER_H