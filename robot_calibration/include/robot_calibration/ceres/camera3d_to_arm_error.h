/*
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

#ifndef ROBOT_CALIBRATION_CERES_CAMERA3D_TO_ARM_ERROR_H
#define ROBOT_CALIBRATION_CERES_CAMERA3D_TO_ARM_ERROR_H

#include <string>
#include <ceres/ceres.h>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

/**
 *  \brief Model of a camera on a kinematic chain.
 */
struct Camera3dToArmError
{
  /**
   *  \brief Error block for calibrating a 3d camera to a single arm.
   *  \param camera_model The model for the camera, used for reprojection.
   *  \param arm_model The model for the arm, used for reprojection.
   *  \param free_param_info Helper container for processing the free parameters.
   *  \param data The calibration data collected
   */
  Camera3dToArmError(Camera3dModel* camera_model,
                     ChainModel* arm_model,
                     CalibrationOffsetParser* offsets,
                     robot_calibration_msgs::CalibrationData& data)
  {
    camera_model_ = camera_model;
    arm_model_ = arm_model;
    offsets_ = offsets;
    data_ = data;
  }

  virtual ~Camera3dToArmError() {}

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

    // Project the camera observations
    std::vector<geometry_msgs::PointStamped> camera_pts =
        camera_model_->project(data_, *offsets_);

    // Project the arm estimation
    std::vector<geometry_msgs::PointStamped> arm_pts =
        arm_model_->project(data_, *offsets_);

    if (camera_pts.size() != arm_pts.size())
    {
      std::cerr << "Camera observation does not match arm estimation in size." << std::endl;
      return false;
    }

    // Compute residuals
    for (size_t i = 0; i < camera_pts.size(); ++i)
    {
      if (camera_pts[i].header.frame_id != arm_pts[i].header.frame_id)
        std::cerr << "Projected observation frame_id does not match projected estimate." << std::endl;
      residuals[(3*i)+0] = camera_pts[i].point.x - arm_pts[i].point.x;
      residuals[(3*i)+1] = camera_pts[i].point.y - arm_pts[i].point.y;
      residuals[(3*i)+2] = camera_pts[i].point.z - arm_pts[i].point.z;
    }

    return true;  // always return true
  }

  /**
   *  \brief Helper factory function to create a new error block. Parameters
   *         are described in the class constructor, which this function calls.
   *  \tparam num_points The number of points in the observation, this forms the
   *          size of the residuals.
   *  \tparam num_free_params The number of free parameters being used for
   *          joint and link calibration.
   */
  static ceres::CostFunction* Create(Camera3dModel* camera_model,
                                     ChainModel* arm_model,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::CalibrationData& data)
  {
    int index = -1;
    for (size_t k = 0; k < data.observations.size() ; k++)
    {
      if ( data.observations[k].sensor_name == camera_model->name())
      {
        index = k;
        break;
      }
    }
    
    if (index == -1)
    {
      std::cerr << "Sensor name doesn't match any of the existing finders" << std::endl;
      return 0;
    }

    ceres::DynamicNumericDiffCostFunction<Camera3dToArmError> * func;
    func = new ceres::DynamicNumericDiffCostFunction<Camera3dToArmError>(
                    new Camera3dToArmError(camera_model, arm_model, offsets, data));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(data.observations[index].features.size() * 3);

    return static_cast<ceres::CostFunction*>(func);
  }

  Camera3dModel * camera_model_;
  ChainModel * arm_model_;
  CalibrationOffsetParser * offsets_;
  robot_calibration_msgs::CalibrationData data_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_CAMERA3D_TO_ARM_ERROR_H
