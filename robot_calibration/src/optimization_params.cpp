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

#include <robot_calibration/ceres/optimization_params.h>

namespace robot_calibration
{

OptimizationParams::OptimizationParams() :
  base_link("base_link")
{
}

bool OptimizationParams::LoadFromROS(rclcpp::Node::SharedPtr node)
{
  base_link = node->declare_parameter<std::string>("base_link", "base_link");
  max_num_iterations = node->declare_parameter<int>("max_num_iterations", 1000);

  free_params = node->declare_parameter<std::vector<std::string>>(
    "free_params", std::vector<std::string>());
    
  free_frames.clear();
  auto free_frame_names = node->declare_parameter<std::vector<std::string>>(
    "free_frames", std::vector<std::string>());
  for (auto name : free_frame_names)
  {
    FreeFrameParams params;
    params.name = name;
    params.x = node->declare_parameter<bool>(name + ".x", false);
    params.y = node->declare_parameter<bool>(name + ".y", false);
    params.z = node->declare_parameter<bool>(name + ".z", false);
    params.roll = node->declare_parameter<bool>(name + ".roll", false);
    params.pitch = node->declare_parameter<bool>(name + ".pitch", false);
    params.yaw = node->declare_parameter<bool>(name + ".yaw", false);
    free_frames.push_back(params);
  }

  free_frames_initial_values.clear();
  free_frame_names = node->declare_parameter<std::vector<std::string>>(
    "free_frames_initial_values", std::vector<std::string>());
  for (auto name : free_frame_names)
  {
    FreeFrameInitialValue params;
    params.name = name;
    params.x = node->declare_parameter<double>(name + ".x", 0.0);
    params.y = node->declare_parameter<double>(name + ".y", 0.0);
    params.z = node->declare_parameter<double>(name + ".z", 0.0);
    params.roll = node->declare_parameter<double>(name + ".roll", 0.0);
    params.pitch = node->declare_parameter<double>(name + ".pitch", 0.0);
    params.yaw = node->declare_parameter<double>(name + ".yaw", 0.0);
    free_frames_initial_values.push_back(params);
  }

  models.clear();
  auto model_names = node->declare_parameter<std::vector<std::string>>(
    "models", std::vector<std::string>());
  for (auto name : model_names)
  {
    Params params;
    params.name = name;
    params.type = node->declare_parameter<std::string>(name + ".type", std::string());
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
    params.type = node->declare_parameter<std::string>(name + ".type", std::string());
    //params.params = error_params[i];
    error_blocks.push_back(params);
  }

  return true;
}

}  // namespace robot_calibration
