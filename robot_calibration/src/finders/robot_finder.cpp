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
#include <robot_calibration/capture/robot_finder.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_finder");

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
                       std::shared_ptr<tf2_ros::Buffer> buffer,
                       rclcpp::Node::WeakPtr weak_node)
{
  if (!PlaneFinder::init(name, buffer, weak_node))
  {
    return false;
  }

  // Get an instance of the node shared pointer
  auto node = weak_node.lock();
  if (!node)
  {
    return false;
  }

  // Name of the sensor model that will be used during optimization
  robot_sensor_name_ = node->declare_parameter<std::string>(name + ".robot_sensor_name", "camera_robot");

  // Publish the observation as a PointCloud2
  robot_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(name + "_robot_points", 10);

  // Valid points must lie within this box, in the transform_frame
  min_robot_x_ = node->declare_parameter<double>(name + ".min_robot_x", -2.0);
  max_robot_x_ = node->declare_parameter<double>(name + ".max_robot_x", 2.0);
  min_robot_y_ = node->declare_parameter<double>(name + ".min_robot_y", -2.0);
  max_robot_y_ = node->declare_parameter<double>(name + ".max_robot_y", 2.0);
  min_robot_z_ = node->declare_parameter<double>(name + ".min_robot_z", 0.0);
  max_robot_z_ = node->declare_parameter<double>(name + ".max_robot_z", 2.0);

  return true;
}

bool RobotFinder::find(robot_calibration_msgs::msg::CalibrationData * msg)
{
  if (!waitForCloud())
  {
    RCLCPP_ERROR(LOGGER, "No point cloud data");
    return false;
  }

  // Remove invalid points
  removeInvalidPoints(cloud_, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);

  // Find the ground plane and extract it
  sensor_msgs::msg::PointCloud2 plane = extractPlane(cloud_);

  // Remove anything that isn't the robot from the cloud
  removeInvalidPoints(cloud_, min_robot_x_, max_robot_x_, min_robot_y_,
                      max_robot_y_, min_robot_z_, max_robot_z_);

  // Pull out both sets of observations
  extractObservation(plane_sensor_name_, plane, msg, publisher_);
  extractObservation(robot_sensor_name_, cloud_, msg, robot_publisher_);

  // Report success
  return true;
}

}  // namespace robot_calibration

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_calibration::RobotFinder, robot_calibration::FeatureFinder)
