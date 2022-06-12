/*
 * Copyright (C) 2018-2022 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_CERES_CALIBRATION_DATA_HELPERS_H
#define ROBOT_CALIBRATION_CERES_CALIBRATION_DATA_HELPERS_H

#include <robot_calibration_msgs/msg/calibration_data.hpp>

namespace robot_calibration
{

/**
 *  @brief Determine which observation index corresponds to a particular sensor name
 */
int getSensorIndex(
  const robot_calibration_msgs::msg::CalibrationData& msg,
  const std::string& sensor)
{
  for (size_t i = 0; i < msg.observations.size(); i++)
  {
    if (msg.observations[i].sensor_name == sensor)
    {
      return i;
    }
  }
  return -1;
}

/**
 *  @brief  Determine if a sample of data has an observation from the desired sensor
 */
bool hasSensor(
  const robot_calibration_msgs::msg::CalibrationData& msg,
  const std::string& sensor)
{
  return getSensorIndex(msg, sensor) >= 0;
}

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_CALIBRATION_DATA_HELPERS_H
