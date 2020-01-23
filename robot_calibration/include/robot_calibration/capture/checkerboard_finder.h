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

#ifndef ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H

#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/plugins/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <ros/ros.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>

namespace robot_calibration
{
/**
 *  \brief This class processes the point cloud input to find a checkerboard
 */
class CheckerboardFinder : public FeatureFinder
{
public:
  CheckerboardFinder();
  bool init(const std::string& name, ros::NodeHandle& n);
  bool find(robot_calibration_msgs::CalibrationData* msg);

private:
  bool findInternal(robot_calibration_msgs::CalibrationData* msg);

  void cameraCallback(const sensor_msgs::PointCloud2& cloud);
  bool waitForCloud();

  bool detectChessBoard(const cv::Mat_<cv::Vec3b>& image, std::vector<cv::Point2f>& points) const;
  bool detectCircleBoard(const cv::Mat_<cv::Vec3b>& image, std::vector<cv::Point2f>& points,
                         const bool asymmetric) const;

  cv::Mat_<cv::Vec3b> getImageFromCloud() const;

  std::vector<geometry_msgs::PointStamped> computeObjectPointsCircleBoard(const bool asymmetric) const;

  std::vector<geometry_msgs::PointStamped> computeObjectPointsChessBoard() const;

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2
  ros::Publisher publisher_;    /// Outgoing sensor_msgs::PointCloud2

  bool waiting_;
  sensor_msgs::PointCloud2 cloud_;
  DepthCameraInfoManager depth_camera_manager_;

  /*
   * ROS Parameters
   */
  int points_x_;  /// Size of checkerboard
  int points_y_;  /// Size of checkerboard

  double square_size_;  /// Size of a square on checkboard (in meters)

  bool output_debug_;  /// Should we output debug image/cloud?

  std::string frame_id_;  /// Name of checkerboard frame

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;

  int32_t trials_;

  /**
   * @brief Either "chess_board, circle_board_symmetric, circle_board_asymmetric"
   */
  static const std::string ChessBoard;
  static const std::string CircleBoardSymmetric;
  static const std::string CircleBoardAsymmetric;
  std::string checkerboard_type_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
