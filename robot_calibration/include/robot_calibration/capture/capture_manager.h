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

#ifndef ROBOT_CALIBRATION_CAPTURE_CAPTURE_MANAGER_H
#define ROBOT_CALIBRATION_CAPTURE_CAPTURE_MANAGER_H

#include <string>
#include <vector>
#include "robot_calibration/capture/chain_manager.h"
#include "robot_calibration/capture/feature_finder_loader.h"

namespace robot_calibration
{
class CaptureManager
{
public:
  CaptureManager();
  bool init(ros::NodeHandle& nh);
  bool moveToState(const sensor_msgs::JointState& state);
  bool captureFeatures(const std::vector<std::string>& feature_names,
                       robot_calibration_msgs::CalibrationData& msg);
  std::string getUrdf();

private:
  ros::Publisher data_pub_;
  ros::Publisher urdf_pub_;
  std_msgs::String description_msg_;

  robot_calibration::ChainManager* chain_manager_;
  robot_calibration::FeatureFinderLoader feature_finder_loader_;
  robot_calibration::FeatureFinderMap finders_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CAPTURE_MANAGER_H
