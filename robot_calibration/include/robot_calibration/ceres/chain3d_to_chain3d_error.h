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

// Author: Michael Ferguson

#ifndef ROBOT_CALIBRATION_CERES_CHAIN3D_TO_CHAIN3D_ERROR_H
#define ROBOT_CALIBRATION_CERES_CHAIN3D_TO_CHAIN3D_ERROR_H

#include <string>
#include <ceres/ceres.h>
#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/ceres/calibration_data_helpers.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

/**
 *  \brief Error block for computing the residual error between two
 *         3d data sources. This can be used to calibrate 3d cameras
 *         to arms, or 3d cameras to other 3d cameras.
 */
struct Chain3dToChain3d
{
  /**
   *  \brief This function is not used direcly, instead use the Create() function.
   *  \param a_model The model for the first chain, used for reprojection.
   *  \param b_model The model for the second chain, used for reprojection.
   *  \param offsets Easy access to the free parameters.
   *  \param data The calibration data collected.
   */
  Chain3dToChain3d(ChainModel* a_model,
                   ChainModel* b_model,
                   CalibrationOffsetParser* offsets,
                   robot_calibration_msgs::CalibrationData& data)
  {
    a_model_ = a_model;
    b_model_ = b_model;
    offsets_ = offsets;
    data_ = data;
  }

  virtual ~Chain3dToChain3d() {}

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
    std::vector<geometry_msgs::PointStamped> a_pts =
        a_model_->project(data_, *offsets_);
    std::vector<geometry_msgs::PointStamped> b_pts =
        b_model_->project(data_, *offsets_);

    if (a_pts.size() != b_pts.size())
    {
      std::cerr << "Observations do not match in size." << std::endl;
      return false;
    }

    // Compute residuals
    for (size_t i = 0; i < a_pts.size(); ++i)
    {
      if (a_pts[i].header.frame_id != b_pts[i].header.frame_id)
        std::cerr << "Projected observation frame_ids do not match." << std::endl;
      residuals[(3*i)+0] = a_pts[i].point.x - b_pts[i].point.x;
      residuals[(3*i)+1] = a_pts[i].point.y - b_pts[i].point.y;
      residuals[(3*i)+2] = a_pts[i].point.z - b_pts[i].point.z;
    }

    return true;  // always return true
  }

  /**
   *  \brief Helper factory function to create a new error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  static ceres::CostFunction* Create(ChainModel* a_model,
                                     ChainModel* b_model,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::CalibrationData& data)
  {
    int index = getSensorIndex(data, a_model->getName());
    if (index == -1)
    {
      // In theory, we should never get here, because the optimizer does a check
      std::cerr << "Sensor name doesn't match any of the existing finders" << std::endl;
      return 0;
    }

    ceres::DynamicNumericDiffCostFunction<Chain3dToChain3d> * func;
    func = new ceres::DynamicNumericDiffCostFunction<Chain3dToChain3d>(
                    new Chain3dToChain3d(a_model, b_model, offsets, data));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(data.observations[index].features.size() * 3);

    return static_cast<ceres::CostFunction*>(func);
  }

  ChainModel * a_model_;
  ChainModel * b_model_;
  CalibrationOffsetParser * offsets_;
  robot_calibration_msgs::CalibrationData data_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_CHAIN3D_TO_CHAIN3D_ERROR_H
