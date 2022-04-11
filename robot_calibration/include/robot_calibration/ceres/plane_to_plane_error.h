/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2015-2017 Fetch Robotics Inc.
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

// Author: Niharika Arora

#ifndef ROBOT_CALIBRATION_CERES_PLANE_TO_PLANE_ERROR_H
#define ROBOT_CALIBRATION_CERES_PLANE_TO_PLANE_ERROR_H

#include <string>
#include <ceres/ceres.h>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/eigen_geometry.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

/**
 *  \brief Error block for computing the fit between two sets of projected
 *         points and their planar models plane (aX + bY + cZ + d = 0).
 *         Typically used to align multiple 3d cameras when they are
 *         not able to see high-resolution checkerboards, but can see
 *         a common plane (wall/floor/etc).
 */
struct PlaneToPlaneError
{
  /**
   *  \brief Error block for calibrating two planar data sets.
   *  \param model_a The model for the first chain, used for planar projection.
   *  \param model_b The model for the second chain, used for planar projection.
   *  \param offsets Easy access to the free parameters.
   *  \param data The calibration data collected.,
   *  \param scale_normal The scaling factor to apply to the residuals of the normal difference.
   *  \param scale_offset The scaling factor to apply to the residual of the offset difference.
   */
  PlaneToPlaneError(ChainModel *model_a,
                    ChainModel *model_b,
                    CalibrationOffsetParser *offsets,
                    robot_calibration_msgs::CalibrationData &data,
                    double scale_normal, double scale_offset)
  {
    model_a_ = model_a;
    model_b_ = model_b;
    offsets_ = offsets;
    data_ = data;
    scale_normal_ = scale_normal;
    scale_offset_ = scale_offset;
  }

  virtual ~PlaneToPlaneError() = default;

  /**
   *  \brief Operator called by CERES optimizer.
   *  \param free_params The offsets to be applied to joints/transforms.
   *  \param residuals The residuals computed, to be returned to the optimizer.
   */
  bool operator()(double const *const *free_params,
                  double *residuals) const
  {
    // Update calibration offsets based on free params
    offsets_->update(free_params[0]);

    // Project the first camera observations
    std::vector<geometry_msgs::PointStamped> a_pts =
        model_a_->project(data_, *offsets_);

    // Get plane parameters for first set of points
    Eigen::MatrixXd matrix_a = getMatrix(a_pts);
    Eigen::Vector3d normal_a;
    double d_a = 0.0;
    getPlane(matrix_a, normal_a, d_a);

    // Project the second camera estimation
    std::vector<geometry_msgs::PointStamped> b_pts =
        model_b_->project(data_, *offsets_);

    // Get plane parameters for second set of points
    Eigen::MatrixXd matrix_b = getMatrix(b_pts);
    Eigen::Vector3d normal_b;
    double d_b = 0.0;
    getPlane(matrix_b, normal_b, d_b);

    // Compute the residuals by minimizing the normals of the calculated planes
    residuals[0] = std::fabs(normal_a(0) - normal_b(0)) * scale_normal_;
    residuals[1] = std::fabs(normal_a(1) - normal_b(1)) * scale_normal_;
    residuals[2] = std::fabs(normal_a(2) - normal_b(2)) * scale_normal_;

    // Final residual is the distance between the centroid of one plane and the second plane itself
    Eigen::Vector3d centroid_a = getCentroid(matrix_a);
    residuals[3] = std::fabs((normal_b(0) * centroid_a(0)) +
                             (normal_b(1) * centroid_a(1)) +
                             (normal_b(2) * centroid_a(2)) + d_b) * scale_offset_;

    return true;  // always return true
  }

  /**
   *  \brief Helper factory function to create a new error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  static ceres::CostFunction *Create(ChainModel *model_a,
                                     ChainModel *model_b,
                                     CalibrationOffsetParser *offsets,
                                     robot_calibration_msgs::CalibrationData &data,
                                     double scale_normal, double scale_offset)
  {
    ceres::DynamicNumericDiffCostFunction <PlaneToPlaneError> *func;
    func = new ceres::DynamicNumericDiffCostFunction<PlaneToPlaneError>(
        new PlaneToPlaneError(model_a, model_b, offsets, data, scale_normal, scale_offset));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(4);

    return static_cast<ceres::CostFunction*>(func);
  }

  ChainModel *model_a_;
  ChainModel *model_b_;
  CalibrationOffsetParser *offsets_;
  robot_calibration_msgs::CalibrationData data_;
  double scale_normal_, scale_offset_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_PLANE_TO_PLANE_ERROR_H
