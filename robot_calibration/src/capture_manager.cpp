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
#include <std_msgs/String.h>
#include "robot_calibration/capture/capture_manager.h"

namespace robot_calibration
{

CaptureManager::CaptureManager()
{
}

bool CaptureManager::init(ros::NodeHandle& nh)
{
  data_pub_ = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);
  urdf_pub_ = nh.advertise<std_msgs::String>("/robot_description", 1, true /* latched */);

  // Create chain manager
  chain_manager_ = new ChainManager(nh);

  // Load feature finders
  if (!feature_finder_loader_.load(nh, finders_))
  {
    ROS_FATAL("Unable to load feature finders!");
    return false;
  }

  // Get the robot_description and republish it
  if (!nh.getParam("/robot_description", description_msg_.data))
  {
    ROS_FATAL("robot_description not set!");
    return false;
  }
  urdf_pub_.publish(description_msg_);

  return true;
}

bool CaptureManager::moveToState(const sensor_msgs::JointState& state)
{
  if (!chain_manager_->moveToState(state))
  {
    return false;
  }

  // Wait for things to settle
  chain_manager_->waitToSettle();
  return true;
}

bool CaptureManager::captureFeatures(const std::vector<std::string>& feature_names,
                                     robot_calibration_msgs::CalibrationData& msg)
{
  for (auto it = finders_.begin(); it != finders_.end(); ++it)
  {
    if (feature_names.empty() ||
        std::find(feature_names.begin(), feature_names.end(), it->first) != feature_names.end())
    {
      if (!it->second->find(&msg))
      {
        ROS_WARN("%s failed to capture features.", it->first.c_str());
        return false;
      }
    }
  }
  chain_manager_->getState(&msg.joint_states);
  // Publish calibration data message.
  data_pub_.publish(msg);
  return true;
}

std::string CaptureManager::getUrdf()
{
  return description_msg_.data;
}

}  // namespace robot_calibration
