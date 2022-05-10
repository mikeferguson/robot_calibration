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

#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

namespace robot_calibration
{
/**
 * @brief Class for moving the base around and calibrating imu and odometry.
 */
class BaseCalibration
{
public:
  /**
   * @brief Create a base calibration instance.
   * @param n NodeHandle at global scope - will subscribe to odom, imu, and base_scan in that namespace.
   */
  BaseCalibration(ros::NodeHandle& n);

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
  void odometryCallback(const nav_msgs::Odometry::Ptr& odom);
  void imuCallback(const sensor_msgs::Imu::Ptr& imu);
  void laserCallback(const sensor_msgs::LaserScan::Ptr& scan);

  /** @brief Send a rotational velocity command. **/
  void sendVelocityCommand(double vel);

  /** @brief Reset the odom/imu counters. */
  void resetInternal();

  ros::Publisher cmd_pub_;

  ros::Subscriber odom_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber scan_subscriber_;

  ros::Time last_odom_stamp_;
  double odom_angle_;

  ros::Time last_imu_stamp_;
  double imu_angle_;

  ros::Time last_scan_stamp_;
  double scan_angle_, scan_r2_, scan_dist_;

  double min_angle_, max_angle_;
  double accel_limit_;
  double align_velocity_, align_gain_, align_tolerance_;

  std::vector<double> scan_;
  std::vector<double> imu_;
  std::vector<double> odom_;

  boost::recursive_mutex data_mutex_;
  bool ready_;
};

}  // namespace robot_calibration
