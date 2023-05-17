/*
 * Copyright (C) 2018-2023 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_COST_FUNCTIONS_CHAIN3D_TO_CAMERA2D_ERROR_HPP
#define ROBOT_CALIBRATION_COST_FUNCTIONS_CHAIN3D_TO_CAMERA2D_ERROR_HPP

#include <string>
#include <ceres/ceres.h>
#include <robot_calibration/optimization/offsets.hpp>
#include <robot_calibration/util/calibration_data.hpp>
#include <robot_calibration/models/camera2d.hpp>
#include <robot_calibration/models/chain3d.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>

namespace robot_calibration
{

/**
 *  \brief Error block for computing the residual error between a
 *         3d chain and a 2d camera.
 */
struct Chain3dToCamera2d
{
  /**
   *  \brief This function is not used direcly, instead use the Create() function.
   *  \param model_3d The model for the 3d chain, used for reprojection.
   *  \param model_2d The model for the 3d camera, used for reverse projection.
   *  \param scale Scalar applied to residual.
   *  \param offsets Easy access to the free parameters.
   *  \param data The calibration data collected.
   */
  Chain3dToCamera2d(Chain3dModel* model_3d,
                    Camera2dModel* model_2d,
                    double scale,
                    OptimizationOffsets* offsets,
                    robot_calibration_msgs::msg::CalibrationData& data)
  {
    model_3d_ = model_3d;
    model_2d_ = model_2d;
    scale_ = scale;
    offsets_ = offsets;
    data_ = data;
  }

  virtual ~Chain3dToCamera2d() {}

  /**
   *  \brief Operator called by CERES optimizer.
   *  \param free_params The offsets to be applied to joints/transforms.
   *  \param residuals The residuals computed, to be returned to the optimizer.
   */
  bool operator()(double const * const * free_params,
                  double* residuals) const
  {
    // Update calibration offsets based on free params
    offsets_->update(free_params[0]);

    // Project the observations into common base frame
    std::vector<geometry_msgs::msg::PointStamped> world_pts =
        model_3d_->project(data_, *offsets_);
    // Now project those 3d points into 2d pixels in the camera model
    std::vector<geometry_msgs::msg::PointStamped> camera_error =
        model_2d_->project_pixel_error(data_, world_pts, *offsets_);

    if (world_pts.size() != camera_error.size())
    {
      std::cerr << "Observations do not match in size." << std::endl;
      return false;
    }

    // Compute residuals
    for (size_t i = 0; i < camera_error.size(); ++i)
    {
      residuals[(2*i)+0] = camera_error[i].point.x * scale_;
      residuals[(2*i)+1] = camera_error[i].point.y * scale_;
    }

    return true;  // always return true
  }

  /**
   *  \brief Helper factory function to create a new error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  static ceres::CostFunction* Create(Chain3dModel* model_3d,
                                     Camera2dModel* model_2d,
                                     double scale,
                                     OptimizationOffsets* offsets,
                                     robot_calibration_msgs::msg::CalibrationData& data)
  {
    int index = getSensorIndex(data, model_3d->getName());
    if (index == -1)
    {
      // In theory, we should never get here, because the optimizer does a check
      std::cerr << "Sensor name doesn't match any of the existing finders" << std::endl;
      return 0;
    }

    ceres::DynamicNumericDiffCostFunction<Chain3dToCamera2d> * func;
    func = new ceres::DynamicNumericDiffCostFunction<Chain3dToCamera2d>(
                    new Chain3dToCamera2d(model_3d, model_2d, scale, offsets, data));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(data.observations[index].features.size() * 2);

    return static_cast<ceres::CostFunction*>(func);
  }

  Chain3dModel * model_3d_;
  Camera2dModel * model_2d_;
  double scale_;
  OptimizationOffsets * offsets_;
  robot_calibration_msgs::msg::CalibrationData data_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_COST_FUNCTIONS_CHAIN3D_TO_CAMERA2D_ERROR_HPP
