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

int main(int argc, char** argv)
{
  ros::init(argc, argv,"base_calibration_node");
  ros::NodeHandle nh;

  // For callbacks
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_calibration::BaseCalibration b(nh);
  b.clearMessages();

  bool verbose = false;
  nh.param<bool>("verbose", verbose, verbose);

  // Rotate at several different speeds
  b.spin(0.5, 1, verbose);
  b.spin(1.5, 1, verbose);
  b.spin(3.0, 2, verbose);
  b.spin(-0.5, 1, verbose);
  b.spin(-1.5, 1, verbose);
  b.spin(-3.0, 2, verbose);

  // TODO: drive towards wall, to calibrate rollout

  // Output yaml file
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
