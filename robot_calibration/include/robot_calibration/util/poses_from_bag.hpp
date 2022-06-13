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

#ifndef ROBOT_CALIBRATION_CAPTURE_POSES_FROM_BAG_H
#define ROBOT_CALIBRATION_CAPTURE_POSES_FROM_BAG_H

#include <string>
#include <vector>
#include <robot_calibration_msgs/msg/capture_config.hpp>

namespace robot_calibration
{

/**
 * @brief Load a vector of calibration poses from a bagfile
 */
bool getPosesFromBag(const std::string& pose_bag_name,
                     std::vector<robot_calibration_msgs::msg::CaptureConfig>& poses);

/*
bool getPosesFromYaml(const std::string& directory_name,
                      std::vector<robot_calibration_msgs::CaptureConfig>& poses);
*/

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_POSES_FROM_BAG_H
