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

#ifndef ROBOT_CALIBRATION_CERES_GROUND_PLANE_ERROR_H
#define ROBOT_CALIBRATION_CERES_GROUND_PLANE_ERROR_H

#include <string>
#include <ceres/ceres.h>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

struct GroundPlaneError
{
  GroundPlaneError(Camera3dModel* camera_model,
                   CalibrationOffsetParser* offsets,
                   robot_calibration_msgs::CalibrationData& data,
                   double z)
  {
    camera_model_ = camera_model;
    z_ = z;
    offsets_ = offsets;
    data_ = data;
  }

  virtual ~GroundPlaneError() {}

  bool operator()(double const * const * free_params,
                  double* residuals) const
  {
    // Update calibration offsets based on free params
    offsets_->update(free_params[0]);

    // Project the camera observations
    std::vector<geometry_msgs::PointStamped> camera_pts =
        camera_model_->project(data_, *offsets_);

    // Compute residuals
    for (size_t i = 0; i < camera_pts.size() ; ++i)
    {
      residuals[i] = camera_pts[i].point.z - z_;  // if camera_pts is in base frame
    }
    return true;
  }

  static ceres::CostFunction* Create(Camera3dModel* camera_model,
                                     double z ,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::CalibrationData& data)
  {
    int index = -1;
    for (size_t k =0; k < data.observations.size() ; k++)
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

    ceres::DynamicNumericDiffCostFunction<GroundPlaneError> * func;
    func = new ceres::DynamicNumericDiffCostFunction<GroundPlaneError>(
                    new GroundPlaneError(camera_model, offsets, data, z));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(data.observations[index].features.size());

    return static_cast<ceres::CostFunction*>(func);
  }

  Camera3dModel * camera_model_;
  double z_;
  CalibrationOffsetParser * offsets_;
  robot_calibration_msgs::CalibrationData data_;
};
}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_GROUND_PLANE_FINDER_H
