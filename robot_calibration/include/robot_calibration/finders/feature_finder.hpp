/*
 * Copyright (C) 2018-2022 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_FINDERS_FEATURE_FINDER_HPP
#define ROBOT_CALIBRATION_FINDERS_FEATURE_FINDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>
#include <tf2_ros/buffer.h>

namespace robot_calibration
{

/**
 *  @brief Base class for a feature finder.
 */
class FeatureFinder
{
public:
  // pluginlib requires empty constructor
  FeatureFinder() {};
  virtual ~FeatureFinder() {};

  /**
   *  @brief Initialize the feature finder.
   *  @param name The name of this finder.
   *  @param node The node to use when loading feature
   *         finder configuration data.
   *  @returns True/False if the feature finder was able to be initialized
   */
  virtual bool init(const std::string& name,
                    std::shared_ptr<tf2_ros::Buffer> buffer,
                    rclcpp::Node::SharedPtr /* node */)
  {
    name_ = name;
    tf2_buffer_ = buffer;
    return true;
  };

  /**
   *  @brief Get the name of this feature finder.
   */
  std::string getName()
  {
    return name_;
  }

  /**
   *  @brief Once the robot has been moved into the proper position
   *         and settled, this function will be called. It should
   *         add any new observations to the msg passed in.
   *  @param msg The message to which observations should be added.
   *  @returns True if feature finder succeeded in finding the
   *           features and adding them to the observation list.
   *           False otherwise.
   */
  virtual bool find(robot_calibration_msgs::msg::CalibrationData * msg) = 0;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

private:
  std::string name_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_FINDERS_FEATURE_FINDER_HPP
