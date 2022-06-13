/*
 * Copyright (C) 2022 Michael Ferguson
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2014 Unbounded Robotics Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace robot_calibration
{
/**
 * @brief Class for moving the base around and calibrating imu and odometry.
 */
class BaseCalibration : public rclcpp::Node
{
public:
  /**
   * @brief Create a base calibration instance.
   */
  BaseCalibration();

  /** @brief Clear any received messages. */
  void clearMessages();

  /** @brief Print out current calibration state. */ 
  std::string print();

  /** @brief Print out the calibration data. */
  std::string printCalibrationData();

  /**
   * @brief Align to the wall.
   * @param Angle angle to align to wall.
   * @param verbose Should the console output be stupidly verbose?
   */
  bool align(double angle, bool verbose = false);

  /** @brief Spin and record imu, odom, scan. */
  bool spin(double velocity, int rotations, bool verbose = false);

private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu);
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

  /** @brief Send a rotational velocity command. **/
  void sendVelocityCommand(double vel);

  /** @brief Reset the odom/imu counters. */
  void resetInternal();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

  rclcpp::Time last_odom_stamp_;
  double odom_angle_;

  rclcpp::Time last_imu_stamp_;
  double imu_angle_;

  rclcpp::Time last_scan_stamp_;
  double scan_angle_, scan_r2_, scan_dist_, r2_tolerance_;

  double min_angle_, max_angle_;
  double accel_limit_;
  double align_velocity_, align_gain_, align_tolerance_;

  std::vector<double> scan_;
  std::vector<double> imu_;
  std::vector<double> odom_;

  std::recursive_mutex data_mutex_;
  bool ready_;
};

}  // namespace robot_calibration
