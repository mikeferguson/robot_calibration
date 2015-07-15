/*
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

#ifndef ROBOT_CALIBRATION_CAMERA_INFO_H
#define ROBOT_CALIBRATION_CAMERA_INFO_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

namespace robot_calibration
{

enum {CAMERA_INFO_P_FX_INDEX=0,
      CAMERA_INFO_P_FY_INDEX=5,
      CAMERA_INFO_P_CX_INDEX=2,
      CAMERA_INFO_P_CY_INDEX=6};

enum {CAMERA_INFO_K_FX_INDEX=0,
      CAMERA_INFO_K_FY_INDEX=4,
      CAMERA_INFO_K_CX_INDEX=2,
      CAMERA_INFO_K_CY_INDEX=5};

enum {CAMERA_INFO_D_1=0,
      CAMERA_INFO_D_2=1,
      CAMERA_INFO_D_3=2,
      CAMERA_INFO_D_4=3,
      CAMERA_INFO_D_5=4};

enum {CAMERA_PARAMS_CX_INDEX=0,
      CAMERA_PARAMS_CY_INDEX=1,
      CAMERA_PARAMS_FX_INDEX=2,
      CAMERA_PARAMS_FY_INDEX=3,
      CAMERA_PARAMS_Z_SCALE_INDEX=4,
      CAMERA_PARAMS_Z_OFFSET_INDEX=5};


/** @brief Update the camera calibration with the new offsets */
inline sensor_msgs::CameraInfo updateCameraInfo(double camera_fx, double camera_fy,
                                                double camera_cx, double camera_cy,
                                                const sensor_msgs::CameraInfo& info)
{
  sensor_msgs::CameraInfo new_info = info;

  new_info.P[CAMERA_INFO_P_CX_INDEX] *= camera_cx + 1.0;  // CX
  new_info.P[CAMERA_INFO_P_CY_INDEX] *= camera_cy + 1.0;  // CY
  new_info.P[CAMERA_INFO_P_FX_INDEX] *= camera_fx + 1.0;  // FX
  new_info.P[CAMERA_INFO_P_FY_INDEX] *= camera_fy + 1.0;  // FY

  new_info.K[CAMERA_INFO_K_CX_INDEX] = new_info.P[CAMERA_INFO_P_CX_INDEX];
  new_info.K[CAMERA_INFO_K_CY_INDEX] = new_info.P[CAMERA_INFO_P_CY_INDEX];
  new_info.K[CAMERA_INFO_K_FX_INDEX] = new_info.P[CAMERA_INFO_P_FX_INDEX];
  new_info.K[CAMERA_INFO_K_FY_INDEX] = new_info.P[CAMERA_INFO_P_FY_INDEX];

  return new_info;
}

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAMERA_INFO_H
