/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_CAPTURE_GRIPPER_DEPTH_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_GRIPPER_DEPTH_FINDER_H

#include <ros/ros.h>
#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/capture/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

namespace robot_calibration
{

class GripperDepthFinder : public FeatureFinder
{

public:
  GripperDepthFinder(ros::NodeHandle & n);

  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  void cameraCallback(const sensor_msgs::ImageConstPtr& cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

  bool waiting_;
  sensor_msgs::ImageConstPtr cloud_;
  DepthCameraInfoManager depth_camera_manager_;

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_GRIPPER_DEPTH_FINDER_H

