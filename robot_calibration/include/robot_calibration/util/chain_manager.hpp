/*
 * Copyright (C) 2022 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_CAPTURE_CHAIN_MANAGER_HPP
#define ROBOT_CALIBRATION_CAPTURE_CHAIN_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>

namespace robot_calibration
{

/**
 * @brief Manages moving joints to a new pose, determining when they
 *        are settled, and returning current joint_states.
 */
class ChainManager
{
  using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
  using MoveGroupAction = moveit_msgs::action::MoveGroup;

  // This controls a single chain
  struct ChainController
  {
    ChainController(rclcpp::Node::SharedPtr node,
                    const std::string& name,
                    const std::string& topic,
                    const std::string& planning_group) :
      chain_name(name),
      chain_planning_group(planning_group)
    {
      client = rclcpp_action::create_client<TrajectoryAction>(node, topic);
    }

    bool shouldPlan()
    {
      return (chain_planning_group != "");
    }

    rclcpp_action::Client<TrajectoryAction>::SharedPtr client;
    std::string chain_name;
    std::string chain_planning_group;
    std::vector<std::string> joint_names;
  };

public:
  /**
   * @brief Constructor, sets up chains from ros parameters.
   * @param node The node handle, sets namespace for parameters.
   * @param wait_time The time to wait for each action to come up.
   */
  ChainManager(rclcpp::Node::SharedPtr node, long int wait_time = 15);

  /**
   * @brief Send commands to all managed joints. The ChainManager automatically
   *        figures out which controller to send these to.
   * @returns False if failed.
   */
  bool moveToState(const sensor_msgs::msg::JointState& state);

  /**
   * @brief Wait for joints to settle.
   */
  bool waitToSettle();

  /**
   * @brief Get the current JointState message.
   */
  bool getState(sensor_msgs::msg::JointState* state);

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
  void stateCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg);

  trajectory_msgs::msg::JointTrajectoryPoint makePoint(const sensor_msgs::msg::JointState& state,
                                                       const std::vector<std::string>& joints);

  // Subscriber for joint_states topic, storage of message
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
  std::mutex state_mutex_;
  sensor_msgs::msg::JointState state_;
  bool state_is_valid_;

  // Mechanisms for passing commands to controllers
  double duration_;
  std::vector<std::shared_ptr<ChainController> > controllers_;
  rclcpp_action::Client<MoveGroupAction>::SharedPtr move_group_;
  double velocity_factor_;  // scaling factor to slow down move_group plans
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHAIN_MANAGER_HPP