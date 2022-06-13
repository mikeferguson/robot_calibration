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

#ifndef ROBOT_CALIBRATION_FINDERS_ROBOT_FINDER_HPP
#define ROBOT_CALIBRATION_FINDERS_ROBOT_FINDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <robot_calibration/finders/plane_finder.hpp>

namespace robot_calibration
{

class RobotFinder : public PlaneFinder
{
public:
  RobotFinder();
  virtual ~RobotFinder() = default;
  virtual bool init(const std::string& name,
                   std::shared_ptr<tf2_ros::Buffer> buffer,
                   rclcpp::Node::SharedPtr node);
  virtual bool find(robot_calibration_msgs::msg::CalibrationData * msg);

protected:
  // Observation name for robot points
  std::string robot_sensor_name_;

  // Publisher for robot points debugging
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr robot_publisher_;

  double min_robot_x_;
  double max_robot_x_;
  double min_robot_y_;
  double max_robot_y_;
  double min_robot_z_;
  double max_robot_z_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_FINDERS_ROBOT_FINDER_HPP
