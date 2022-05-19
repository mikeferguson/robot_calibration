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

#include <cmath>
#include <fstream>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "robot_calibration/calibration/base_calibration.h"

#define PI          3.14159265359

namespace robot_calibration
{
BaseCalibration::BaseCalibration(ros::NodeHandle& n) : ready_(false)
{
  // Setup times
  last_odom_stamp_ = last_imu_stamp_ = last_scan_stamp_ = ros::Time::now();

  // Get params
  ros::NodeHandle nh("~");

  // Min/Max acceptable error to continue aligning with wall
  nh.param<double>("min_angle", min_angle_, -0.5);
  nh.param<double>("max_angle", max_angle_, 0.5);

  // How fast to accelerate
  nh.param<double>("accel_limit", accel_limit_, 2.0);
  // Maximum velocity to command base during alignment
  nh.param<double>("align_velocity", align_velocity_, 0.2);
  // Gain to turn alignment error into velocity
  nh.param<double>("align_gain", align_gain_, 2.0);
  // Tolerance when aligning the base
  nh.param<double>("align_tolerance", align_tolerance_, 0.2);
  // Tolerance for r2
  nh.param<double>("r2_tolerance", r2_tolerance_, 0.1);

  // Command publisher
  cmd_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Subscribe
  odom_subscriber_ = n.subscribe("odom", 5, &BaseCalibration::odometryCallback, this);
  imu_subscriber_ =  n.subscribe("imu", 5, &BaseCalibration::imuCallback, this);
  scan_subscriber_ = n.subscribe("base_scan", 1, &BaseCalibration::laserCallback, this);

  resetInternal();
}

void BaseCalibration::clearMessages()
{
  scan_.clear();
  odom_.clear();
  imu_.clear();
}

std::string BaseCalibration::print()
{
  std::stringstream ss;
  ss << scan_r2_ << " " << imu_angle_ << " " << odom_angle_ << " " << scan_angle_;
  return ss.str();
}

std::string BaseCalibration::printCalibrationData()
{
  double odom, imu;
  ros::NodeHandle nh;
  nh.param<double>("base_controller/track_width", odom, 0.37476);
  nh.param<double>("imu/gyro/scale", imu, 0.001221729);

  // Scaling to be computed
  double odom_scale = 0.0;
  double imu_scale = 0.0;

  // Get sum
  for (size_t i = 0; i < scan_.size(); ++i)
  {
    odom_scale += (scan_[i] - odom_[i]) / odom_[i];
    imu_scale += (scan_[i] - imu_[i]) / imu_[i];
  }
  // Divide sum by size
  odom_scale /= scan_.size();
  imu_scale /= scan_.size();
  // Output odom/imu values
  std::stringstream ss;
  ss << "odom: " << odom * (1.0 + odom_scale) << std::endl;
  ss << "imu: " << imu * (1.0 + imu_scale) << std::endl;
  return ss.str();
}

bool BaseCalibration::align(double angle, bool verbose)
{
  while (!ready_)
  {
    ROS_WARN("Not ready!");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  std::cout << "aligning..." << std::endl;

  double error = scan_angle_ - angle;
  while (fabs(error) > align_tolerance_ || (scan_r2_ < r2_tolerance_))
  {
    if (verbose)
    {
      std::cout << scan_r2_ << " " << scan_angle_ << std::endl;
    }

    // Send command
    double velocity = std::min(std::max(-error * align_gain_, -align_velocity_), align_velocity_);
    sendVelocityCommand(velocity);

    // Sleep a moment
    ros::Duration(0.02).sleep();
    ros::spinOnce();

    // Update error before comparing again
    error = scan_angle_ - angle;

    // Exit if shutting down
    if (!ros::ok())
    {
      sendVelocityCommand(0.0);
      return false;
    }
  }

  // Done - stop the robot
  sendVelocityCommand(0.0);
  std::cout << "...done" << std::endl;
  ros::Duration(0.25).sleep();

  return true;
}

bool BaseCalibration::spin(double velocity, int rotations, bool verbose)
{
  double scan_start = scan_angle_;

  // Align straight at the wall
  align(0.0, verbose);
  resetInternal();
  std::cout << "spin..." << std::endl;

  // Need to account for de-acceleration time (v^2/2a)
  double angle = rotations * 2 * PI - (0.5 * velocity * velocity / accel_limit_);

  while (fabs(odom_angle_) < angle)
  {
    if (verbose)
    {
      std::cout << scan_angle_ << " " << odom_angle_ << " " << imu_angle_ << std::endl;
    }
    sendVelocityCommand(velocity);
    ros::Duration(0.02).sleep();
    ros::spinOnce();

    // Exit if shutting down
    if (!ros::ok())
    {
      sendVelocityCommand(0.0);
      return false;
    }
  }

  // Stop the robot
  sendVelocityCommand(0.0);
  std::cout << "...done" << std::endl;

  // Wait to stop
  ros::Duration(0.5 + fabs(velocity) / accel_limit_).sleep();

  // Save measurements
  imu_.push_back(imu_angle_);
  odom_.push_back(odom_angle_);
  if (velocity > 0)
  {
    scan_.push_back(scan_start + 2 * rotations * PI - scan_angle_);
  }
  else
  {
    scan_.push_back(scan_start - 2 * rotations * PI - scan_angle_);
  }

  return true;
}

void BaseCalibration::odometryCallback(const nav_msgs::Odometry::Ptr& odom)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  double dt = (odom->header.stamp - last_odom_stamp_).toSec();
  odom_angle_ += odom->twist.twist.angular.z * dt;

  last_odom_stamp_ = odom->header.stamp;
}

void BaseCalibration::imuCallback(const sensor_msgs::Imu::Ptr& imu)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  double dt = (imu->header.stamp - last_imu_stamp_).toSec();
  imu_angle_ += imu->angular_velocity.z * dt;

  last_imu_stamp_ = imu->header.stamp;
}

void BaseCalibration::laserCallback(const sensor_msgs::LaserScan::Ptr& scan)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  double angle = scan->angle_min;
  double mean_x, mean_y, n;
  mean_x = mean_y = n = 0;
  int start = -1;
  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
  {
    if (angle < min_angle_ || angle > max_angle_)
    {
      continue;
    }

    if (std::isnan(scan->ranges[i]))
    {
      continue;
    }

    if (start < 0)
    {
      start = i;
    }

    // Compute point
    double px = sin(angle) * scan->ranges[i];
    double py = cos(angle) * scan->ranges[i];

    mean_x += px;
    mean_y += py;
    ++n;
  }

  if (n == 0)
  {
    return;
  }

  mean_x /= n;
  mean_y /= n;

  angle = scan->angle_min + start * scan->angle_increment;  // reset angle
  double x, y, xx, xy, yy;
  x = y = xx = xy = yy = n = 0;
  for (size_t i = start; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
  {
    if (angle > max_angle_)
    {
      break;
    }

    if (std::isnan(scan->ranges[i]))
    {
      continue;
    }

    // Compute point
    double px = sin(angle) * scan->ranges[i] - mean_x;
    double py = cos(angle) * scan->ranges[i] - mean_y;

    // Sums for simple linear regression
    xx += px * px;
    xy += px * py;
    x += px;
    y += py;
    yy += py * py;
    ++n;
  }

  scan_dist_ = mean_y;
  scan_angle_ = atan2((n*xy-x*y)/(n*xx-x*x), 1.0);
  scan_r2_ = fabs(xy)/(xx * yy);
  last_scan_stamp_ = scan->header.stamp;
  ready_ = true;
}

void BaseCalibration::sendVelocityCommand(double vel)
{
  geometry_msgs::Twist twist;
  twist.angular.z = vel;
  cmd_pub_.publish(twist);
}

void BaseCalibration::resetInternal()
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);
  odom_angle_ = imu_angle_ = scan_angle_ = scan_r2_ = 0.0;
}

}  // namespace robot_calibration
