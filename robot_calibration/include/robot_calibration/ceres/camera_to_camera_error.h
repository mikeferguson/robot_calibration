/*
 * Copyright (C) 2015-2017 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless repts_matuired by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Niharika Arora

#ifndef ROBOT_CALIBRATION_CERES_CAMERA_TO_CAMERA_ERROR_H
#define ROBOT_CALIBRATION_CERES_CAMERA_TO_CAMERA_ERROR_H

#include <string>
#include <ceres/ceres.h>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

namespace robot_calibration
{

/**
 *  \brief Model of a camera on another camera chain.
 */
struct CameraToCameraError
{
  /**
   *  \brief Error block for calibrating two cameras with respect to each other.
   *  \param camera1_model The model for the first camera, used for reprojection.
   *  \param camera2_model The model for the second camera, used for reprojection.
   *  \param free_param_info Helper container for processing the free parameters.
   *  \param data The calibration data collected
   */
  CameraToCameraError(Camera3dModel *camera1_model,
      Camera3dModel *camera2_model,
      CalibrationOffsetParser *offsets,
      robot_calibration_msgs::CalibrationData &data)
  {
    camera1_model_ = camera1_model;
    camera2_model_ = camera2_model;
    offsets_ = offsets;
    data_ = data;
  }

  virtual ~CameraToCameraError()
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
    std::vector <geometry_msgs::PointStamped> camera1_pts =
        camera1_model_->project(data_, *offsets_);

    // Project the second camera estimation
    std::vector <geometry_msgs::PointStamped> camera2_pts =
        camera2_model_->project(data_, *offsets_);

    // Calculate plane normals using SVD
    std::vector <cv::Point3f> camera1_points;
    std::vector <cv::Point3f> camera2_points;

    for (size_t i = 0; i < camera1_pts.size(); ++i)
    {
      camera1_points.push_back(cv::Point3f(camera1_pts[i].point.x, camera1_pts[i].point.y, camera1_pts[i].point.z));
    }
    for (size_t i = 0; i < camera2_pts.size(); ++i)
    {
      camera2_points.push_back(cv::Point3f(camera2_pts[i].point.x, camera2_pts[i].point.y, camera2_pts[i].point.z));
    }

    // Find the planes
    cv::Mat plane_1 = getPlane(camera1_points);
    cv::Mat plane_2 = getPlane(camera2_points);

    // Compute the residuals by minimizing the normals of the calculated planes
    residuals[0] = std::fabs(plane_1.at<float>(0, 0)) - std::fabs(plane_2.at<float>(0, 0));
    residuals[1] = std::fabs(plane_1.at<float>(0, 1)) - std::fabs(plane_2.at<float>(0, 1));
    residuals[2] = std::fabs(plane_1.at<float>(0, 2)) - std::fabs(plane_2.at<float>(0, 2));

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
  static ceres::CostFunction *Create(Camera3dModel *camera1_model,
      Camera3dModel *camera2_model,
      CalibrationOffsetParser *offsets,
       robot_calibration_msgs::CalibrationData &data)
  {
    int index = -1;
    for (size_t k = 0; k < data.observations.size(); k++)
    {
      if (data.observations[k].sensor_name == camera2_model->name())
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

    ceres::DynamicNumericDiffCostFunction <CameraToCameraError> *func;
    func = new ceres::DynamicNumericDiffCostFunction<CameraToCameraError>(
        new CameraToCameraError(camera1_model, camera2_model, offsets, data));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(3);

    return static_cast<ceres::CostFunction *>(func);
  }

  /**
   *  \brief Does a plane fit to the poinst and calculates the normals
   *  \tparam points The points sampled from the point cloud
   *  \treturn The normals
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

  Camera3dModel *camera1_model_;
  Camera3dModel *camera2_model_;
  CalibrationOffsetParser *offsets_;
  robot_calibration_msgs::CalibrationData data_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_CAMERA_TO_CAMERA_ERROR_H
