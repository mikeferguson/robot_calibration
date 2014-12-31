/*
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

#ifndef UBR_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H_
#define UBR_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ubr_calibration/capture/feature_finder.h>
#include <ubr_calibration/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <ubr_calibration/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

namespace ubr_calibration
{

/**
 *  \brief This class processes the point cloud input to find a checkerboard
 */
class CheckerboardFinder : public FeatureFinder
{
public:
  CheckerboardFinder(ros::NodeHandle & n);

  /**
   * \brief Attempts to find the checkerboard incoming data.
   * \param msg CalibrationData instance to fill in with point information.
   * \param points_x Number of checkerboard points in x
   * \param points_y Number of checkerboard points in y
   * \returns True if point has been filled in.
   */
  bool find(ubr_calibration::CalibrationData * msg);

private:
  bool findInternal(ubr_calibration::CalibrationData * msg);

  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  int points_x_;
  int points_y_;

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
};

}  // namespace ubr_calibration

#endif  // UBR_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H_
