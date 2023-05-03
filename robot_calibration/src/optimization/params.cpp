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
  rclcpp::Logger logger = node->get_logger();

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
    RCLCPP_INFO(logger, "Adding free frame: %s", name.c_str());
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
    parameter_ns + ".free_frames_initial_values", std::vector<std::string>());
  for (auto name : free_frame_names)
  {
    RCLCPP_INFO(logger, "Adding initial values for: %s", name.c_str());
    std::string prefix = parameter_ns + "." + name + "_initial_values";
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
    parameter_ns + ".models", std::vector<std::string>());
  for (auto name : model_names)
  {
    RCLCPP_INFO(logger, "Adding model: %s", name.c_str());
    ModelParams params;
    params.name = name;
    params.type = node->declare_parameter<std::string>(parameter_ns + "." + name + ".type", std::string());
    params.frame = node->declare_parameter<std::string>(parameter_ns + "." + name + ".frame", std::string());
    params.param_name = node->declare_parameter<std::string>(parameter_ns + "." + name + ".param_name", std::string());
    models.push_back(params);
  }

  error_blocks.clear();
  auto error_block_names = node->declare_parameter<std::vector<std::string>>(
    parameter_ns + ".error_blocks", std::vector<std::string>());
  for (auto name : error_block_names)
  {
    std::string prefix = parameter_ns + "." + name;
    std::string type = node->declare_parameter<std::string>(prefix + ".type", std::string());
    RCLCPP_INFO(logger, "Adding %s: %s", type.c_str(), name.c_str());

    if (type == "chain3d_to_chain3d")
    {
      std::shared_ptr<Chain3dToChain3dParams> params = std::make_shared<Chain3dToChain3dParams>();
      params->name = name;
      params->type = type;
      params->model_a = node->declare_parameter<std::string>(prefix + ".model_a", std::string());
      params->model_b = node->declare_parameter<std::string>(prefix + ".model_b", std::string());
      error_blocks.push_back(params);
    }
    else if (type == "chain3d_to_plane")
    {
      std::shared_ptr<Chain3dToPlaneParams> params = std::make_shared<Chain3dToPlaneParams>();
      params->name = name;
      params->type = type;
      params->model = node->declare_parameter<std::string>(prefix + ".model", std::string());
      params->a = node->declare_parameter<double>(prefix + ".a", 0.0);
      params->b = node->declare_parameter<double>(prefix + ".b", 0.0);
      params->c = node->declare_parameter<double>(prefix + ".c", 1.0);
      params->d = node->declare_parameter<double>(prefix + ".d", 0.0);
      params->scale = node->declare_parameter<double>(prefix + ".scale", 1.0);
      error_blocks.push_back(params);
    }
    else if (type == "chain3d_to_mesh")
    {
      std::shared_ptr<Chain3dToMeshParams> params = std::make_shared<Chain3dToMeshParams>();
      params->name = name;
      params->type = type;
      params->model = node->declare_parameter<std::string>(prefix + ".model", std::string());
      params->link_name = node->declare_parameter<std::string>(prefix + ".link_name", std::string());
      error_blocks.push_back(params);
    }
    else if (type == "plane_to_plane")
    {
      std::shared_ptr<PlaneToPlaneParams> params = std::make_shared<PlaneToPlaneParams>();
      params->name = name;
      params->type = type;
      params->model_a = node->declare_parameter<std::string>(prefix + ".model_a", std::string());
      params->model_b = node->declare_parameter<std::string>(prefix + ".model_b", std::string());
      params->normal_scale = node->declare_parameter<double>(prefix + ".normal_scale", 1.0);
      params->offset_scale = node->declare_parameter<double>(prefix + ".offset_scale", 1.0);
      error_blocks.push_back(params);
    }
    else if (type == "outrageous")
    {
      std::shared_ptr<OutrageousParams> params = std::make_shared<OutrageousParams>();
      params->name = name;
      params->type = type;
      params->param = node->declare_parameter<std::string>(prefix + ".param", std::string());
      params->joint_scale = node->declare_parameter<double>(prefix + ".joint_scale", 1.0);
      params->position_scale = node->declare_parameter<double>(prefix + ".position_scale", 1.0);
      params->rotation_scale = node->declare_parameter<double>(prefix + ".rotation_scale", 1.0);
      error_blocks.push_back(params);
    }
    else
    {
      RCLCPP_ERROR(logger, "Error block %s of type '%s' is unrecognized", name.c_str(), type.c_str());
    }
  }

  if (error_blocks.empty())
  {
    RCLCPP_ERROR(logger, "No error_blocks are defined!");
  }

  return true;
}

}  // namespace robot_calibration
