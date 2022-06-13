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

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

namespace robot_calibration
{

/** @brief Class to hold parameters for optimization. */
struct OptimizationParams
{
  struct FreeFrameParams
  {
    std::string name;
    bool x;
    bool y;
    bool z;
    bool roll;
    bool pitch;
    bool yaw;
  };

  struct FreeFrameInitialValue
  {
    std::string name;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };

  struct Params
  {
    std::string name;
    std::string type;
  };

  std::string base_link;
  std::vector<std::string> free_params;
  std::vector<FreeFrameParams> free_frames;
  std::vector<FreeFrameInitialValue> free_frames_initial_values;
  std::vector<Params> models;
  std::vector<Params> error_blocks;

  // Parameters for the optimizer itself
  int max_num_iterations;

  OptimizationParams();

  /**
   * @brief Load from ROS parameters
   * @param node Node pointer to use for declaring/loading parameters
   * @param parameter_ns Namespace for optimization parameters
   */
  bool LoadFromROS(rclcpp::Node::SharedPtr node, const std::string& parameter_ns);

  template<typename T>
  T getParam(Params& params, const std::string& name, T default_value)
  {
    return default_value;
    //if (!params.params.hasMember(name))
    //{
      //RCLCPP_WARN(LOGGER, name << " was not set, using default of " << default_value);
      //return default_value;
    //}

    //return static_cast<T>(params.params[name]);
  }
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_OPTIMIZATION_PARAMS_HPP