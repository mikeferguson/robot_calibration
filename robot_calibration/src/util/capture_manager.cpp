/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2015 Fetch Robotics Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_calibration/util/capture_manager.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("capture_manager");

namespace robot_calibration
{

CaptureManager::CaptureManager()
{
  description_valid_ = false;
}

bool CaptureManager::init(rclcpp::Node::SharedPtr node)
{
  // Publish calibration data (to be recorded by rosbag)
  data_pub_ = node->create_publisher<robot_calibration_msgs::msg::CalibrationData>("/calibration_data", 10);

  // Subscribe to robot_description
  urdf_sub_ = node->create_subscription<std_msgs::msg::String>("/robot_description",
    rclcpp::QoS(1).transient_local(),
    std::bind(&CaptureManager::callback, this, std::placeholders::_1));

  // Create chain manager
  chain_manager_ = new ChainManager(node);

  // Load feature finders
  if (!feature_finder_loader_.load(node, finders_))
  {
    RCLCPP_FATAL(LOGGER, "Unable to load feature finders!");
    return false;
  }

  return true;
}

bool CaptureManager::moveToState(const sensor_msgs::msg::JointState& state)
{
  if (!chain_manager_->moveToState(state))
  {
    return false;
  }

  // Wait for things to settle
  chain_manager_->waitToSettle();
  return true;
}

bool CaptureManager::captureFeatures(const std::vector<std::string>& feature_names,
                                     robot_calibration_msgs::msg::CalibrationData& msg)
{
  for (auto it = finders_.begin(); it != finders_.end(); ++it)
  {
    if (feature_names.empty() ||
        std::find(feature_names.begin(), feature_names.end(), it->first) != feature_names.end())
    {
      RCLCPP_INFO(LOGGER, "Capturing features from %s", it->first.c_str());
      if (!it->second->find(&msg))
      {
        RCLCPP_WARN(LOGGER, "%s failed to capture features.", it->first.c_str());
        return false;
      }
    }
  }
  chain_manager_->getState(&msg.joint_states);
  // Publish calibration data message.
  data_pub_->publish(msg);
  return true;
}

void CaptureManager::callback(std_msgs::msg::String::ConstSharedPtr msg)
{
  description_ = msg->data;
  description_valid_ = true;
}

std::string CaptureManager::getUrdf()
{
  while (!description_valid_ && rclcpp::ok())
  {
    RCLCPP_WARN(LOGGER, "Waiting for robot_description");
    rclcpp::sleep_for(std::chrono::seconds(5));
  }
  return description_;
}

}  // namespace robot_calibration
