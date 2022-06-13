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

#ifndef ROBOT_CALIBRATION_CAPTURE_CAPTURE_MANAGER_H
#define ROBOT_CALIBRATION_CAPTURE_CAPTURE_MANAGER_H

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "robot_calibration/capture/chain_manager.h"
#include "robot_calibration/capture/feature_finder_loader.h"

namespace robot_calibration
{
class CaptureManager
{
public:
  CaptureManager();
  bool init(rclcpp::Node::SharedPtr node);
  bool moveToState(const sensor_msgs::msg::JointState& state);
  bool captureFeatures(const std::vector<std::string>& feature_names,
                       robot_calibration_msgs::msg::CalibrationData& msg);
  std::string getUrdf();

private:
  void callback(std_msgs::msg::String::ConstSharedPtr msg);

  rclcpp::Publisher<robot_calibration_msgs::msg::CalibrationData>::SharedPtr data_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  std::string description_;
  bool description_valid_;

  robot_calibration::ChainManager* chain_manager_;
  robot_calibration::FeatureFinderLoader feature_finder_loader_;
  robot_calibration::FeatureFinderMap finders_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CAPTURE_MANAGER_H
