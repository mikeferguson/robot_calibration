/*
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

#define PI          3.14159265359

class BaseCalibration
{
public:
  BaseCalibration(ros::NodeHandle& n) : ready_(false)
  {
    // Setup times
    last_odom_stamp_ = last_imu_stamp_ = last_scan_stamp_ = ros::Time::now();

    // Get params
    ros::NodeHandle nh("~");

    // Min/Max acceptable error when aligning with wall
    nh.param<double>("min_angle", min_angle_, -0.5);
    nh.param<double>("max_angle", max_angle_, 0.5);

    // How fast to accelerate
    nh.param<double>("accel_limit", accel_limit_, 2.0);

    // cmd vel publisher
    cmd_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Subscribe
    odom_subscriber_ = n.subscribe("odom", 5, &BaseCalibration::odometryCallback, this);
    imu_subscriber_ =  n.subscribe("imu", 5, &BaseCalibration::imuCallback, this);
    scan_subscriber_ = n.subscribe("base_scan", 1, &BaseCalibration::laserCallback, this);

    resetInternal();
  }

  void reset()
  {
    scan_.clear();
    odom_.clear();
    imu_.clear();
  }

  std::string print()
  {
    std::stringstream ss;
    ss << scan_r2_ << " " << imu_angle_ << " " << odom_angle_ << " " << scan_angle_;
    return ss.str();
  }

  std::string printCalibrationData()
  {
    double odom, imu;
    ros::NodeHandle nh;
    nh.param<double>("base_controller/track_width", odom, 0.37476);
    nh.param<double>("imu/gyro/scale", imu, 0.001221729);

    // scaling to be computed
    double odom_scale = 0.0;
    double imu_scale = 0.0;

    // get sum
    for (size_t i = 0; i < scan_.size(); ++i)
    {
      odom_scale += (scan_[i] - odom_[i]) / odom_[i];
      imu_scale += (scan_[i] - imu_[i]) / imu_[i];
    }
    // divide sum by size
    odom_scale /= scan_.size();
    imu_scale /= scan_.size();
    // output odom/imu values
    std::stringstream ss;
    ss << "odom: " << odom * (1.0 + odom_scale) << std::endl;
    ss << "imu: " << imu * (1.0 + imu_scale) << std::endl;
    return ss.str();
  }

  /** \brief Align to the wall */
  bool align(bool verbose = false)
  {
    while (!ready_)
      ros::Duration(0.1).sleep();

    std::cout << "aligning..." << std::endl;

    double velocity = 0.2;
    if (scan_angle_ < 0)
      velocity = -0.2;

    while (fabs(scan_angle_) > 0.2 || (scan_r2_ < 0.1))
    {
      if (verbose)
        std::cout << scan_r2_ << " " << scan_angle_ << std::endl;
      sendVelocityCommand(velocity);
      ros::Duration(0.02).sleep();
    }
    sendVelocityCommand(0.0);
    std::cout << "...done" << std::endl;
    ros::Duration(0.25).sleep();

    return true;
  }

  /** \brief Spin and record imu, odom, scan */
  bool spin(double velocity, int rotations, bool verbose = false)
  {
    double scan_start = scan_angle_;

    align();
    resetInternal();
    std::cout << "spin..." << std::endl;

    // need to account for de-acceleration time (v^2/2a)
    double angle = rotations * 2 * PI - (0.5 * velocity * velocity / accel_limit_);

    while (fabs(odom_angle_) < angle)
    {
      if (verbose)
        std::cout << scan_angle_ << " " << odom_angle_ << " " << imu_angle_ << std::endl;
      sendVelocityCommand(velocity);
      ros::Duration(0.02).sleep();
    }
    sendVelocityCommand(0.0);
    std::cout << "...done" << std::endl;

    // wait to stop
    ros::Duration(0.5 + fabs(velocity) / accel_limit_).sleep();

    // save measurements
    imu_.push_back(imu_angle_);
    odom_.push_back(odom_angle_);
    if (velocity > 0)
      scan_.push_back(scan_start + 2 * rotations * PI - scan_angle_);
    else
      scan_.push_back(scan_start - 2 * rotations * PI - scan_angle_);

    return true;
  }

private:
  void odometryCallback(const nav_msgs::Odometry::Ptr& odom)
  {
    boost::recursive_mutex::scoped_lock lock(data_mutex_);

    double dt = (odom->header.stamp - last_odom_stamp_).toSec();
    odom_angle_ += odom->twist.twist.angular.z * dt;

    last_odom_stamp_ = odom->header.stamp;
  }

  void imuCallback(const sensor_msgs::Imu::Ptr& imu)
  {
    boost::recursive_mutex::scoped_lock lock(data_mutex_);

    double dt = (imu->header.stamp - last_imu_stamp_).toSec();
    imu_angle_ += imu->angular_velocity.z * dt;

    last_imu_stamp_ = imu->header.stamp;
  }

  void laserCallback(const sensor_msgs::LaserScan::Ptr& scan)
  {
    boost::recursive_mutex::scoped_lock lock(data_mutex_);

    double angle = scan->angle_min;
    double mean_x, mean_y, n;
    mean_x = mean_y = n = 0;
    int start = -1;
    for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
    {
      if (angle < min_angle_ || angle > max_angle_)
        continue;
      if (start < 0)
        start = i;

      // Compute point
      double px = sin(angle) * scan->ranges[i];
      double py = cos(angle) * scan->ranges[i];

      mean_x += px;
      mean_y += py;
      ++n;
    }

    if (n == 0)
      return;

    mean_x /= n;
    mean_y /= n;

    angle = scan->angle_min + start * scan->angle_increment;  // reset angle
    double x, y, xx, xy, yy;
    x = y = xx = xy = yy = n = 0;
    for (size_t i = start; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
    {
      if (angle > max_angle_)
        break;

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

  // Send a rotational velocity command
  void sendVelocityCommand(double vel)
  {
    geometry_msgs::Twist twist;
    twist.angular.z = vel;
    cmd_pub_.publish(twist);
  }

  // reset the odom/imu counters
  void resetInternal()
  {
    boost::recursive_mutex::scoped_lock lock(data_mutex_);
    odom_angle_ = imu_angle_ = scan_angle_ = scan_r2_ = 0.0;
  }

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

  std::vector<double> scan_;
  std::vector<double> imu_;
  std::vector<double> odom_;

  boost::recursive_mutex data_mutex_;
  bool ready_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv,"base_calibration_node");
  ros::NodeHandle nh;

  // for callbacks
  ros::AsyncSpinner spinner(1);
  spinner.start();

  BaseCalibration b(nh);
  b.reset();

  // rotate at several different speeds
  b.spin(0.5, 1);
  b.spin(1.5, 1);
  b.spin(3.0, 2);
  b.spin(-0.5, 1);
  b.spin(-1.5, 1);
  b.spin(-3.0, 2);

  // TODO: drive towards wall, to calibrate rollout

  // output yaml file
  {
    // Generate datecode
    char datecode[80];
    {
      std::time_t t = std::time(NULL);
      std::strftime(datecode, 80, "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
    }
    std::stringstream yaml_name;
    yaml_name << "/tmp/base_calibration_" << datecode << ".yaml";
    std::ofstream file;
    file.open(yaml_name.str().c_str());
    std::string cal = b.printCalibrationData();
    file << cal;
    file.close();
    std::cout << cal;
  }

  spinner.stop();
  return 0;
}
