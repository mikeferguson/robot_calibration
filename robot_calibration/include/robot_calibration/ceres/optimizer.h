/*
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

#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/ceres/optimization_params.h>
#include <robot_calibration/ceres/camera3d_to_arm_error.h>
#include <robot_calibration/ceres/ground_plane_error.h>
#include <robot_calibration/ceres/data_functions.h>
#include <robot_calibration/ceres/outrageous_error.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <boost/shared_ptr.hpp>
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
               std::vector<robot_calibration_msgs::CalibrationData> data,
               bool progress_to_stdout = false);

  /** \brief Returns the summary of the optimization last run. */
  boost::shared_ptr<ceres::Solver::Summary> summary()
  {
    return summary_;
  }

  boost::shared_ptr<CalibrationOffsetParser> getOffsets()
  {
    return offsets_;
  }

private:
  urdf::Model model_;
  std::string root_frame_;
  std::string led_frame_;
  KDL::Tree tree_;

  std::map<std::string, ChainModel*> models_;

  boost::shared_ptr<CalibrationOffsetParser> offsets_;
  boost::shared_ptr<ceres::Solver::Summary> summary_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_OPTIMIZER_H
