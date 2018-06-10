/*
 * Copyright (C) 2018 Michael Ferguson
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
#include <robot_calibration/ceres/calibration_data_helpers.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

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

  virtual ~PlaneToPlaneError()
  {}

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
    std::vector <geometry_msgs::PointStamped> a_pts =
        model_a_->project(data_, *offsets_);

    // Project the second camera estimation
    std::vector <geometry_msgs::PointStamped> b_pts =
        model_b_->project(data_, *offsets_);

    // Calculate plane normals using SVD
    std::vector <cv::Point3f> a_points;
    std::vector <cv::Point3f> b_points;

    for (size_t i = 0; i < a_pts.size(); ++i)
    {
      a_points.push_back(cv::Point3f(a_pts[i].point.x, a_pts[i].point.y, a_pts[i].point.z));
    }
    for (size_t i = 0; i < b_pts.size(); ++i)
    {
      b_points.push_back(cv::Point3f(b_pts[i].point.x, b_pts[i].point.y, b_pts[i].point.z));
    }

    // Find the planes
    cv::Mat plane_1 = getPlane(a_points);
    cv::Mat plane_2 = getPlane(b_points);

    // Compute the residuals by minimizing the normals of the calculated planes
    residuals[0] = (std::fabs(plane_1.at<float>(0, 0)) - std::fabs(plane_2.at<float>(0, 0))) * scale_normal_;
    residuals[1] = (std::fabs(plane_1.at<float>(0, 1)) - std::fabs(plane_2.at<float>(0, 1))) * scale_normal_;
    residuals[2] = (std::fabs(plane_1.at<float>(0, 2)) - std::fabs(plane_2.at<float>(0, 2))) * scale_normal_;
    //residuals[3] = TODO

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
    int index = getSensorIndex(data, model_a->name());
    if (index == -1)
    {
      // In theory, we should never get here, because the optimizer does a check
      std::cerr << "Sensor name doesn't match any of the existing finders" << std::endl;
      return 0;
    }

    ceres::DynamicNumericDiffCostFunction <PlaneToPlaneError> *func;
    func = new ceres::DynamicNumericDiffCostFunction<PlaneToPlaneError>(
        new PlaneToPlaneError(model_a, model_b, offsets, data, scale_normal, scale_offset));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(4);

    return static_cast<ceres::CostFunction *>(func);
  }

  /**
   *  \brief Does a plane fit to the points and calculates the normals
   *  \param points The points sampled from the point cloud
   *  \return The normals
   */
  cv::Mat getPlane(std::vector <cv::Point3f> points) const
  {
    // Calculate centroid
    cv::Point3f centroid(0, 0, 0);
    for (size_t i = 0; i < points.size(); i++)
    {
      centroid += points.at(i);
    }

    centroid.x = centroid.x / points.size();
    centroid.y = centroid.y / points.size();
    centroid.z = centroid.z / points.size();

    // subtract centroid from all points
    cv::Mat pts_mat(points.size(), 3, CV_32F);
    for (size_t i = 0; i < points.size(); i++)
    {
      pts_mat.at<float>(i, 0) = points[i].x - centroid.x;
      pts_mat.at<float>(i, 1) = points[i].y - centroid.y;
      pts_mat.at<float>(i, 2) = points[i].z - centroid.z;
    }

    // SVD
    cv::SVD svd(pts_mat, cv::SVD::FULL_UV);
    cv::Mat normal = svd.vt.row(2);

    return normal;
  }

  ChainModel *model_a_;
  ChainModel *model_b_;
  CalibrationOffsetParser *offsets_;
  robot_calibration_msgs::CalibrationData data_;
  double scale_normal_, scale_offset_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_PLANE_TO_PLANE_ERROR_H
