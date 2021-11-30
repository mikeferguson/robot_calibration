/*
 * Copyright (C) 2018 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_LOAD_BAG_H
#define ROBOT_CALIBRATION_LOAD_BAG_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <boost/foreach.hpp>  // for rosbag iterator

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

/**
 *  \brief Load a bagfile of calibration data.
 *  \param file_name Name of the bag file to load.
 *  \param description_msg This will be loaded with the URDF string.
 *  \param data This will be loaded with the calibration data.
 */
bool load_bag(const std::string& file_name,
              std_msgs::String& description_msg,
              std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  // Open the bag file
  rosbag::Bag bag_;
  try
  {
    bag_.open(file_name, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException&)
  {
    ROS_FATAL_STREAM("Cannot open " << file_name);
    return false;
  }

  // Get robot_description from bag file
  rosbag::View model_view_(bag_, rosbag::TopicQuery("/robot_description"));
  if (model_view_.size() < 1)
  {
    ROS_FATAL_STREAM("robot_description topic not found in bag file.");
    return false;
  }
  std_msgs::String::ConstPtr description_ = model_view_.begin()->instantiate<std_msgs::String>();
  description_msg = *description_;

  // Parse calibration_data topic
  rosbag::View data_view_(bag_, rosbag::TopicQuery("/calibration_data"));
  BOOST_FOREACH (rosbag::MessageInstance const m, data_view_)
  {
    robot_calibration_msgs::CalibrationData::ConstPtr msg = m.instantiate<robot_calibration_msgs::CalibrationData>();
    data.push_back(*msg);
  }

  return true;
}

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_LOAD_BAG_H
