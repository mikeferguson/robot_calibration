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

#ifndef ROBOT_CALIBRATION_CALIBRATION_EXPORT_H
#define ROBOT_CALIBRATION_CALIBRATION_EXPORT_H

#include <string>
#include <robot_calibration/ceres/optimizer.h>

namespace robot_calibration
{
/**
 * @brief Write the outputs of a calibration
 * @param optimizer The optimizer instance, where we get our offsets from
 * @param initial_urdf The initial URDF, to which offsets are added
 * @param data The raw calibration data, currently used only to get CameraInfo
 */
bool exportResults(Optimizer& optimizer, const std::string& initial_urdf,
                   const std::vector<robot_calibration_msgs::msg::CalibrationData>& data);

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CALIBRATION_EXPORT_H