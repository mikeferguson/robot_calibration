/*
 * Copyright (C) 2018-2023 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_FINDERS_CHECKERBOARD_FINDER_HPP
#define ROBOT_CALIBRATION_FINDERS_CHECKERBOARD_FINDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <robot_calibration/finders/feature_finder.hpp>
#include <robot_calibration/util/depth_camera_info.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

namespace robot_calibration
{

/**
 *  \brief Finds checkerboards in images or point clouds
 */
template <typename T>
class CheckerboardFinder : public FeatureFinder
{
public:
  CheckerboardFinder();
  bool init(const std::string& name,
            std::shared_ptr<tf2_ros::Buffer> buffer,
            rclcpp::Node::SharedPtr node);
  bool find(robot_calibration_msgs::msg::CalibrationData * msg);

private:
  bool findInternal(robot_calibration_msgs::msg::CalibrationData * msg);
  bool findCheckerboardPoints(sensor_msgs::msg::Image::SharedPtr image,
                              std::vector<cv::Point2f>& points);

  void cameraCallback(typename T::ConstSharedPtr cloud);
  bool waitForMsg();

  typename rclcpp::Subscription<T>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  bool waiting_;
  T msg_;
  DepthCameraInfoManager depth_camera_manager_;

  /*
   * ROS Parameters
   */
  int points_x_;        /// Size of checkerboard
  int points_y_;        /// Size of checkerboard

  double square_size_;     /// Size of a square on checkboard (in meters)

  bool output_debug_;   /// Should we output debug image/cloud?

  std::string frame_id_;   /// Name of checkerboard frame

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_FINDERS_CHECKERBOARD_FINDER_HPP
