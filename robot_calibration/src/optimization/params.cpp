/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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

#include <robot_calibration/optimization/params.hpp>

namespace robot_calibration
{

OptimizationParams::OptimizationParams() :
  base_link("base_link")
{
}

bool OptimizationParams::LoadFromROS(rclcpp::Node::SharedPtr node,
                                     const std::string& parameter_ns)
{
  // Base link should be consistent across all calibration steps
  base_link = node->declare_parameter<std::string>("base_link", "base_link");

  max_num_iterations = node->declare_parameter<int>(
    parameter_ns + ".max_num_iterations", 1000);

  free_params = node->declare_parameter<std::vector<std::string>>(
    parameter_ns + ".free_params", std::vector<std::string>());
    
  free_frames.clear();
  auto free_frame_names = node->declare_parameter<std::vector<std::string>>(
    parameter_ns + ".free_frames", std::vector<std::string>());
  for (auto name : free_frame_names)
  {
    std::string prefix = parameter_ns + "." + name;
    FreeFrameParams params;
    params.name = name;
    params.x = node->declare_parameter<bool>(prefix + ".x", false);
    params.y = node->declare_parameter<bool>(prefix + ".y", false);
    params.z = node->declare_parameter<bool>(prefix + ".z", false);
    params.roll = node->declare_parameter<bool>(prefix + ".roll", false);
    params.pitch = node->declare_parameter<bool>(prefix + ".pitch", false);
    params.yaw = node->declare_parameter<bool>(prefix + ".yaw", false);
    free_frames.push_back(params);
  }

  free_frames_initial_values.clear();
  free_frame_names = node->declare_parameter<std::vector<std::string>>(
    "free_frames_initial_values", std::vector<std::string>());
  for (auto name : free_frame_names)
  {
    std::string prefix = parameter_ns + "." + name;
    FreeFrameInitialValue params;
    params.name = name;
    params.x = node->declare_parameter<double>(prefix + ".x", 0.0);
    params.y = node->declare_parameter<double>(prefix + ".y", 0.0);
    params.z = node->declare_parameter<double>(prefix + ".z", 0.0);
    params.roll = node->declare_parameter<double>(prefix + ".roll", 0.0);
    params.pitch = node->declare_parameter<double>(prefix + ".pitch", 0.0);
    params.yaw = node->declare_parameter<double>(prefix + ".yaw", 0.0);
    free_frames_initial_values.push_back(params);
  }

  models.clear();
  auto model_names = node->declare_parameter<std::vector<std::string>>(
    "models", std::vector<std::string>());
  for (auto name : model_names)
  {
    Params params;
    params.name = name;
    params.type = node->declare_parameter<std::string>(parameter_ns + "." + name + ".type", std::string());
    //params.params = model_params[i];
    models.push_back(params);
  }

  error_blocks.clear();
  auto error_block_names = node->declare_parameter<std::vector<std::string>>(
    "error_blocks", std::vector<std::string>());
  for (auto name : error_block_names)
  {
    Params params;
    params.name = name;
    params.type = node->declare_parameter<std::string>(parameter_ns + "." + name + ".type", std::string());
    //params.params = error_params[i];
    error_blocks.push_back(params);
  }

  return true;
}

}  // namespace robot_calibration
