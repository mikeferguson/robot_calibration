/*
 * Copyright (C) 2018-2022 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_UTIL_CALIBRATION_DATA_HPP
#define ROBOT_CALIBRATION_UTIL_CALIBRATION_DATA_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>

namespace robot_calibration
{

/**
 *  @brief Determine which observation index corresponds to a particular sensor name
 */
inline int getSensorIndex(
  const robot_calibration_msgs::msg::CalibrationData& msg,
  const std::string& sensor)
{
  for (size_t i = 0; i < msg.observations.size(); i++)
  {
    if (msg.observations[i].sensor_name == sensor)
    {
      return i;
    }
  }
  return -1;
}

/**
 *  @brief Determine if a sample of data has an observation from the desired sensor
 */
inline bool hasSensor(
  const robot_calibration_msgs::msg::CalibrationData& msg,
  const std::string& sensor)
{
  return getSensorIndex(msg, sensor) >= 0;
}

/**
 *  \brief Load a bagfile of calibration data.
 *  \param file_name Name of the bag file to load.
 *  \param description_msg This will be loaded with the URDF string.
 *  \param data This will be loaded with the calibration data.
 */
inline bool load_bag(const std::string& file_name,
                     std_msgs::msg::String& description_msg,
                     std::vector<robot_calibration_msgs::msg::CalibrationData>& data)
{
  rosbag2_cpp::Reader reader;
  
  // Open the bag file
  reader.open(file_name);

  // Load messages
  while (reader.has_next())
  {
    auto bag_message = reader.read_next();

    if (bag_message->topic_name == "/robot_description")
    {
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      rclcpp::Serialization<std_msgs::msg::String> serialization;
      serialization.deserialize_message(&extracted_serialized_msg, &description_msg);
    }
    else if (bag_message->topic_name == "/calibration_data")
    {
      robot_calibration_msgs::msg::CalibrationData msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      rclcpp::Serialization<robot_calibration_msgs::msg::CalibrationData> serialization;
      serialization.deserialize_message(&extracted_serialized_msg, &msg);
      data.push_back(msg);
    }
  }

  return true;
}

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_UTIL_CALIBRATION_DATA_HPP
