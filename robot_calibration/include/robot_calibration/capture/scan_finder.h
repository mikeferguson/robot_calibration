/*
 * Copyright (C) 2022 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_CAPTURE_SCAN_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_SCAN_FINDER_H

#include <rclcpp/rclcpp.hpp>
#include <robot_calibration/plugins/feature_finder.h>
#include <robot_calibration_msgs/msg/calibration_data.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace robot_calibration
{
/**
 * @brief The scan finder is useful for aligning a laser scanner
 *        with other sensors. In particular, the laser scan be
 *        multiplied in the Z direction in order to create a plane
 *        for plane to plane calibration.
 */
class ScanFinder : public FeatureFinder
{
public:
  ScanFinder();
  virtual ~ScanFinder() = default;
  virtual bool init(const std::string& name,
                    std::shared_ptr<tf2_ros::Buffer> buffer,
                    rclcpp::Node::SharedPtr node);
  virtual bool find(robot_calibration_msgs::msg::CalibrationData * msg);

protected:
  /**
   * @brief ROS callback - updates scan_ and resets waiting_ to false
   */
  virtual void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  /**
   * @brief Wait until a new scan has arrived
   */
  virtual bool waitForScan();

  /**
   * @brief Extract a point cloud from laser scan points that meet criteria
   */
  void extractPoints(sensor_msgs::msg::PointCloud2& cloud);

  /**
   * @brief Extract the point cloud into a calibration message.
   */
  void extractObservation(const sensor_msgs::msg::PointCloud2& cloud,
                          robot_calibration_msgs::msg::CalibrationData * msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  rclcpp::Clock::SharedPtr clock_;

  bool waiting_;
  sensor_msgs::msg::LaserScan scan_;

  std::string laser_sensor_name_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  int z_repeats_;
  double z_offset_;
  std::string transform_frame_;

  bool output_debug_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_SCAN_FINDER_H
