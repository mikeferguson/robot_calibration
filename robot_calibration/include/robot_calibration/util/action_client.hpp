/*
 * Copyright (C) 2022 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_UTIL_ACTION_CLIENT_HPP
#define ROBOT_CALIBRATION_UTIL_ACTION_CLIENT_HPP

#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace robot_calibration
{
// Goal States
enum class ActionClientState
{
  ACTIVE,
  SUCCEEDED,
  ABORTED,
};

template <typename ActionType>
class ActionClient
{
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionType>;
  using ActionWrappedResult = typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult;
  using ActionGoal =  typename ActionType::Goal;
  using ActionResult = typename ActionType::Result::SharedPtr;

public:
  ActionClient() : state_(ActionClientState::ABORTED)
  {
  }

  bool init(rclcpp::Node::SharedPtr node,
            const std::string& name)
  {
    node_ = node;
    name_ = name;
    client_ = rclcpp_action::create_client<ActionType>(node, name);
    return true;
  }

  bool waitForServer(double timeout)
  {
    auto node = node_.lock();
    if (!node)
    {
      // Can't even really warn here...
      return false;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for %s...", name_.c_str());
    return client_->wait_for_action_server(std::chrono::milliseconds(static_cast<int>(1000 * timeout)));
  }

  void sendGoal(const typename ActionType::Goal& goal)
  {
    auto goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
    goal_options.result_callback =
      std::bind(&ActionClient<ActionType>::resultCallback, this, std::placeholders::_1);
    client_->async_send_goal(goal, goal_options);
    state_ = ActionClientState::ACTIVE;
  }

  ActionClientState waitForResult(rclcpp::Duration timeout)
  {
    // Make sure we can lock the node
    auto node = node_.lock();
    if (!node)
    {
      // Can't even really warn here...
      return state_;
    }

    // Wait for result, or timeout
    rclcpp::Time start = node->now();
    while (state_ == ActionClientState::ACTIVE)
    {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(10));

      if ((node->now() - start) > timeout)
      {
        RCLCPP_WARN(node->get_logger(), "Timed out waiting for action result");
        return state_;
      }
    }
  
    return state_;
  }

  ActionResult getResult()
  {
    return result_;
  }

private:
  void resultCallback(const ActionWrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      state_ = ActionClientState::SUCCEEDED;
    }
    else
    {
      state_ = ActionClientState::ABORTED;
    }
    result_ = result.result;
  }

  std::string name_;
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<rclcpp_action::Client<ActionType>> client_;
  ActionClientState state_;
  ActionResult result_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_UTIL_ACTION_CLIENT_HPP
