/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2015-2017 Fetch Robotics Inc.
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

// Author: Niharika Arora, Michael Ferguson

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <robot_calibration/capture/robot_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::RobotFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

RobotFinder::RobotFinder() :
  PlaneFinder()
{
}

bool RobotFinder::init(const std::string& name,
                       ros::NodeHandle & nh)
{
  if (!PlaneFinder::init(name, nh))
    return false;

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("robot_sensor_name", robot_sensor_name_, "camera_robot");

  // Publish the observation as a PointCloud2
  robot_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_robot_points", 10);

  // Valid points must lie within this box, in the transform_frame
  nh.param<double>("min_robot_x", min_robot_x_, -2.0);
  nh.param<double>("max_robot_x", max_robot_x_, 2.0);
  nh.param<double>("min_robot_y", min_robot_y_, -2.0);
  nh.param<double>("max_robot_y", max_robot_y_, 2.0);
  nh.param<double>("min_robot_z", min_robot_z_, 0.0);
  nh.param<double>("max_robot_z", max_robot_z_, 2.0);

  return true;
}

bool RobotFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  if (!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  // Remove invalid points
  removeInvalidPoints(cloud_, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);

  // Find the ground plane and extract it
  sensor_msgs::PointCloud2 plane = extractPlane(cloud_);

  // Remove anything that isn't the robot from the cloud
  removeInvalidPoints(cloud_, min_robot_x_, max_robot_x_, min_robot_y_,
                      max_robot_y_, min_robot_z_, max_robot_z_);

  // Pull out both sets of observations
  extractObservation(plane_sensor_name_, plane, msg, &publisher_);
  extractObservation(robot_sensor_name_, cloud_, msg, &robot_publisher_);

  // Report success
  return true;
}

}  // namespace robot_calibration
