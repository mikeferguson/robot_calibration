/*
 * Copyright (C) 2018 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_CERES_CHAIN3D_TO_PLANE_ERROR_H
#define ROBOT_CALIBRATION_CERES_CHAIN3D_TO_PLANE_ERROR_H

#include <string>
#include <math.h>
#include <ceres/ceres.h>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/ceres/calibration_data_helpers.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

/**
 *  \brief Error block for computing the fit between a set of projected
 *         points and a plane (aX + bY + cZ + d = 0). Typically used
 *         to align sensor with the ground, but could be used to align
 *         with a flat robot base, etc.
 */
struct Chain3dToPlane
{
  /**
   *  \brief This function is not used direcly, instead use the Create() function.
   *  \param chain_model The model for the chain, used for reprojection.
   *  \param offsets Easy access to the free parameters.
   *  \param data The calibration data collected.
   *  \param a The plane parameter a.
   *  \param b The plane parameter b.
   *  \param c The plane parameter c.
   *  \param d The plane parameter d.
   *  \param scale The scaling factor to apply to residual of distance to plane.
   */
  Chain3dToPlane(ChainModel* chain_model,
                 CalibrationOffsetParser* offsets,
                 robot_calibration_msgs::CalibrationData& data,
                 double a, double b, double c, double d,
                 double scale)
  {
    chain_model_ = chain_model;
    offsets_ = offsets;
    data_ = data;

    a_ = a;
    b_ = b;
    c_ = c;
    d_ = d;

    // Precompute denominator of distance to plane
    double denom = sqrt((a_ * a_) + (b_* b_) + (c_ * c_));
    if (abs(denom) < 0.1)
    {
      std::cerr << "Plane normal is extremely small: " << denom << std::endl;
    }
    scale_ = scale / denom;
  }

  virtual ~Chain3dToPlane() {}

  bool operator()(double const * const * free_params,
                  double* residuals) const
  {
    // Update calibration offsets based on free params
    offsets_->update(free_params[0]);

    // Project the camera observations
    std::vector<geometry_msgs::PointStamped> chain_pts =
        chain_model_->project(data_, *offsets_);

    // Compute residuals
    for (size_t i = 0; i < chain_pts.size() ; ++i)
    {
      residuals[i] = abs((a_ * chain_pts[i].point.x) +
                         (b_ * chain_pts[i].point.y) +
                         (c_ * chain_pts[i].point.z) + d_) * scale_;
    }
    return true;
  }

  /**
   *  \brief Helper factory function to create a new error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  static ceres::CostFunction* Create(ChainModel* a_model,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::CalibrationData& data,
                                     double a, double b, double c, double d,
                                     double scale)
  {
    int index = getSensorIndex(data, a_model->name());
    if (index == -1)
    {
      // In theory, we should never get here, because the optimizer does a check
      std::cerr << "Sensor name doesn't match any of the existing finders" << std::endl;
      return 0;
    }

    ceres::DynamicNumericDiffCostFunction<Chain3dToPlane> * func;
    func = new ceres::DynamicNumericDiffCostFunction<Chain3dToPlane>(
                    new Chain3dToPlane(a_model, offsets, data, a, b, c, d, scale));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(data.observations[index].features.size());

    return static_cast<ceres::CostFunction*>(func);
  }

  ChainModel * chain_model_;
  CalibrationOffsetParser * offsets_;
  robot_calibration_msgs::CalibrationData data_;
  double a_, b_, c_, d_;
  double scale_, denom_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_CHAIN3D_TO_PLANE_ERROR_H
