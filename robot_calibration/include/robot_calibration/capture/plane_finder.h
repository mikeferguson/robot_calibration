/*
 * Copyright (C) 2014-2017 Fetch Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_CAPTURE_PLANE_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_PLANE_FINDER_H

#include <ros/ros.h>
#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/plugins/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>

namespace robot_calibration
{

class PlaneFinder : public FeatureFinder
{
public:
  PlaneFinder();
  bool init(const std::string& name, ros::NodeHandle & n, bool const head_driver = true);
  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  void cameraCallback(const sensor_msgs::PointCloud2& cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  bool waiting_;
  sensor_msgs::PointCloud2 cloud_;
  DepthCameraInfoManager depth_camera_manager_;

  std::string camera_sensor_name_;
  double points_max_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  std::string transform_frame_;

  bool output_debug_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_PLANE_FINDER_H
