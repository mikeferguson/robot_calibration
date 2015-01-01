/*
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

#ifndef ROBOT_CALIBRATION_CERES_OPTIMIZATION_PARAMS_H
#define ROBOT_CALIBRATION_CERES_OPTIMIZATION_PARAMS_H

#include <ros/ros.h>

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

  struct Params
  {
    std::string name;
    std::string type;
    XmlRpc::XmlRpcValue params;
  };

  std::string base_link;
  std::vector<std::string> free_params;
  std::vector<FreeFrameParams> free_frames;
  std::vector<Params> models;
  std::vector<Params> error_blocks;

  OptimizationParams();
  bool LoadFromROS(ros::NodeHandle& nh);
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_OPTIMIZATION_PARAMS_H