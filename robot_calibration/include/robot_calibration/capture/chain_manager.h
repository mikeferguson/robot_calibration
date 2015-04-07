/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/MoveGroupAction.h>

namespace robot_calibration
{

/**
 * @brief Manages moving joints to a new pose, determining when they
 *        are settled, and returning current joint_states.
 */
class ChainManager
{
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;
  typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupClient;
  typedef boost::shared_ptr<MoveGroupClient> MoveGroupClientPtr;
  typedef boost::shared_ptr<const moveit_msgs::MoveGroupResult> MoveGroupResultPtr;

  // This controls a single chain
  struct ChainController
  {
    ChainController(const std::string& name,
                    const std::string& topic,
                    const std::string& planning_group) :
      client(topic, true),
      chain_name(name),
      chain_planning_group(planning_group)
    {
    }

    bool shouldPlan()
    {
      return (chain_planning_group != "");
    }

    TrajectoryClient client;
    std::string chain_name;
    std::string chain_planning_group;
    std::vector<std::string> joint_names;
  };

public:
  /**
   * @brief Constructor, sets up chains from ros parameters.
   * @param nh The node handle, sets namespace for parameters.
   * @param wait_time The time to wait for each action to come up.
   */
  ChainManager(ros::NodeHandle& nh, double wait_time = 15.0);

  /**
   * @brief Send commands to all managed joints. The ChainManager automatically
   *        figures out which controller to send these to.
   * @returns False if failed.
   */
  bool moveToState(const sensor_msgs::JointState& state);

  /**
   * @brief Wait for joints to settle.
   */
  bool waitToSettle();

  /**
   * @brief Get the current JointState message.
   */
  bool getState(sensor_msgs::JointState* state);

  /**
   * @brief Get the names of chains. Mainly for testing
   */
  std::vector<std::string> getChains();

  /**
   * @brief Get the joint names associated with a chain. Mainly for testing
   */
  std::vector<std::string> getChainJointNames(const std::string& chain_name);

  // Mainly for testing
  std::string getPlanningGroupName(const std::string& chain_name);

private:
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  trajectory_msgs::JointTrajectoryPoint makePoint(const sensor_msgs::JointState& state,
                                                  const std::vector<std::string>& joints);

  ros::Subscriber subscriber_;
  sensor_msgs::JointState state_;
  double duration_;
  boost::mutex state_mutex_;
  std::vector<boost::shared_ptr<ChainController> > controllers_;
  MoveGroupClientPtr move_group_;
  double velocity_factor_;  // scaling factor to slow down move_group plans
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHAIN_MANAGER_H