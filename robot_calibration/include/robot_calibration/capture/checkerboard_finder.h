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

#ifndef ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/capture/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <robot_calibration/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

namespace robot_calibration
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
  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  bool findInternal(robot_calibration_msgs::CalibrationData * msg);

  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2
  ros::Publisher publisher_;   /// Outgoing sensor_msgs::PointCloud2

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
  DepthCameraInfoManager depth_camera_manager_;

  /*
   * ROS Parameters
   */
  int points_x_;        /// Size of checkerboard
  int points_y_;        /// Size of checkerboard

  bool output_debug_;   /// Should we output debug image/cloud?
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
