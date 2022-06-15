/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2014 Fetch Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_OPTIMIZATION_PARAMS_HPP
#define ROBOT_CALIBRATION_OPTIMIZATION_PARAMS_HPP

#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

namespace robot_calibration
{

/** @brief Class to hold parameters for optimization. */
struct OptimizationParams
{
  struct Params
  {
    std::string name;
    std::string type;

    virtual ~Params() { }
  };

  using ParamsPtr = std::shared_ptr<Params>;

  struct FreeFrameParams : Params
  {
    bool x;
    bool y;
    bool z;
    bool roll;
    bool pitch;
    bool yaw;
  };

  struct FreeFrameInitialValue : Params
  {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };

  struct ModelParams : Params
  {
    // All models need a sensor frame
    std::string frame;
    // Some models can "mirror" another model's optimization parameters
    std::string param_name;
  };

  struct Chain3dToChain3dParams : Params
  {
    // Chain3d or Camera3d models to use
    std::string model_a;
    std::string model_b;
  };

  struct Chain3dToPlaneParams : Params
  {
    // Chain3d model to use
    std::string model;
    // Parameters for the plane: ax + by + cz + d = 0
    double a, b, c, d;
    // Scalar applied to residual
    double scale;
  };

  struct Chain3dToMeshParams : Params
  {
    // Chain3d model to use
    std::string model;
    // Link in URDF to use for mesh
    std::string link_name;
  };

  struct PlaneToPlaneParams : Params
  {
    // Chain3d or Camera3d models to use
    std::string model_a;
    std::string model_b;
    // Scalars applied to residuals
    double normal_scale;
    double offset_scale;
  };

  struct OutrageousParams : Params
  {
    std::string param;
    double joint_scale;
    double position_scale;
    double rotation_scale;
  };

  std::string base_link;
  std::vector<std::string> free_params;
  std::vector<FreeFrameParams> free_frames;
  std::vector<FreeFrameInitialValue> free_frames_initial_values;
  std::vector<ModelParams> models;
  std::vector<ParamsPtr> error_blocks;

  // Parameters for the optimizer itself
  int max_num_iterations;

  OptimizationParams();

  /**
   * @brief Load from ROS parameters
   * @param node Node pointer to use for declaring/loading parameters
   * @param parameter_ns Namespace for optimization parameters
   */
  bool LoadFromROS(rclcpp::Node::SharedPtr node, const std::string& parameter_ns);
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_OPTIMIZATION_PARAMS_HPP
