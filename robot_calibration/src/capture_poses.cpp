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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include "robot_calibration/capture/poses.h"

namespace robot_calibration
{

// Load a set of calibration poses
bool getPosesFromBag(const std::string& pose_bag_name,
                     std::vector<robot_calibration_msgs::CaptureConfig>& poses)
{
  poses.clear();

  ROS_INFO_STREAM("Opening " << pose_bag_name);
  rosbag::Bag bag;
  try
  {
    bag.open(pose_bag_name, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException&)
  {
    ROS_FATAL_STREAM("Cannot open " << pose_bag_name);
    return false;
  }
  rosbag::View data_view(bag, rosbag::TopicQuery("calibration_joint_states"));

  BOOST_FOREACH (rosbag::MessageInstance const m, data_view)
  {
    robot_calibration_msgs::CaptureConfig::ConstPtr msg = m.instantiate<robot_calibration_msgs::CaptureConfig>();
    if (msg == NULL)
    {
      // Try to load older style bags
      sensor_msgs::JointState::ConstPtr js_msg = m.instantiate<sensor_msgs::JointState>();
      if (js_msg != NULL)
      {
        robot_calibration_msgs::CaptureConfig config;
        config.joint_states = *js_msg;
        poses.push_back(config);
      }
    }
    else
    {
      poses.push_back(*msg);
    }
  }
  return true;
}

}  // namespace robot_calibration
