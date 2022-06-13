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
#include <rosbag2_cpp/reader.hpp>
#include "robot_calibration/capture/poses.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_calibration");

namespace robot_calibration
{

// Load a set of calibration poses
bool getPosesFromBag(const std::string& pose_bag_name,
                     std::vector<robot_calibration_msgs::msg::CaptureConfig>& poses)
{
  poses.clear();

  RCLCPP_INFO(LOGGER, "Opening %s", pose_bag_name.c_str());
  rosbag2_cpp::Reader reader;
  reader.open(pose_bag_name);
  while (reader.has_next())
  {
    try
    {
      auto bag_message = reader.read_next();
      robot_calibration_msgs::msg::CaptureConfig msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      rclcpp::Serialization<robot_calibration_msgs::msg::CaptureConfig> serialization;
      serialization.deserialize_message(&extracted_serialized_msg, &msg);
      poses.push_back(msg);
    }
    catch (std::runtime_error&)
    {
      RCLCPP_WARN(LOGGER, "Unable to deserialize message");
    }
  }
  return true;
}

}  // namespace robot_calibration
