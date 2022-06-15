/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2014 Fetch Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_CERES_OPTIMIZER_H
#define ROBOT_CALIBRATION_CERES_OPTIMIZER_H

#include <memory>
#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/logger.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>
#include <robot_calibration/optimization/offsets.hpp>
#include <robot_calibration/optimization/params.hpp>
#include <robot_calibration/models/camera3d.hpp>
#include <robot_calibration/models/chain3d.hpp>
#include <robot_calibration/util/mesh_loader.hpp>
#include <string>
#include <map>

namespace robot_calibration
{

/** @brief Class to do optimization. */
class Optimizer
{
public:
  /** @brief Standard constructor */
  Optimizer(const std::string& robot_description);
  virtual ~Optimizer();

  /**
   * @brief Run optimization.
   * @param data The data to be used for the optimization. Typically parsed
   *        from bag file, or loaded over some topic subscriber.
   * @param progress_to_stdout If true, Ceres optimizer will output info to
   *        stdout.
   */
  int optimize(OptimizationParams& params,
               std::vector<robot_calibration_msgs::msg::CalibrationData> data,
               rclcpp::Logger& logger,
               bool progress_to_stdout = false);

  /**
   * @brief Returns the summary of the optimization last run.
   */
  std::shared_ptr<ceres::Solver::Summary> summary()
  {
    return summary_;
  }

  std::shared_ptr<OptimizationOffsets> getOffsets()
  {
    return offsets_;
  }

  int getNumParameters()
  {
    return num_params_;
  }

  int getNumResiduals()
  {
    return num_residuals_;
  }

  /**
   * @brief Get the names of all camera models.
   *
   * This is mainly used when deciding what camera_info to export.
   */
  std::vector<std::string> getCameraNames();

private:
  std::shared_ptr<urdf::Model> model_;
  std::string root_frame_;
  std::string led_frame_;
  KDL::Tree tree_;

  std::shared_ptr<MeshLoader> mesh_loader_;

  std::map<std::string, Chain3dModel*> models_;

  std::shared_ptr<OptimizationOffsets> offsets_;
  std::shared_ptr<ceres::Solver::Summary> summary_;

  int num_params_, num_residuals_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_OPTIMIZER_H
