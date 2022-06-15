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
#include <robot_calibration/util/poses_from_yaml.hpp>
#include <yaml-cpp/yaml.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_calibration");

namespace robot_calibration
{

// Load a set of calibration poses
bool getPosesFromYaml(const std::string& filename,
                      std::vector<robot_calibration_msgs::msg::CaptureConfig>& poses)
{
  poses.clear();

  RCLCPP_INFO(LOGGER, "Opening %s", filename.c_str());
  YAML::Node yaml_poses = YAML::LoadFile(filename);

  for (auto pose : yaml_poses)
  {
    robot_calibration_msgs::msg::CaptureConfig msg;

    for (auto it = pose.begin(); it != pose.end(); ++it)
    {
      if (it->first.as<std::string>() == "joints")
      {
        for (auto joint = it->second.begin(); joint != it->second.end(); ++joint)
        {
          msg.joint_states.name.push_back(joint->as<std::string>());
        }
      }
      else if (it->first.as<std::string>() == "positions")
      {
        for (auto position = it->second.begin(); position != it->second.end(); ++position)
        {
          msg.joint_states.position.push_back(position->as<double>());
        }
      }
      else if (it->first.as<std::string>() == "features")
      {
        for (auto feature = it->second.begin(); feature != it->second.end(); ++feature)
        {
          msg.features.push_back(feature->as<std::string>());
        }
      }
    }

    if (!msg.joint_states.name.empty() &&
        msg.joint_states.name.size() == msg.joint_states.position.size())
    {
      poses.push_back(msg);
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Discarding pose due to invalid joint_states");
    }
  }

  return true;
}

}  // namespace robot_calibration
