/*
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

#ifndef ROBOT_CALIBRATION_MODELS_CAMERA3D_H
#define ROBOT_CALIBRATION_MODELS_CAMERA3D_H

#include <robot_calibration/camera_info.h>
#include <robot_calibration/models/chain.h>

namespace robot_calibration
{

/**
 *  \brief Model of a camera on a kinematic chain.
 */
class Camera3dModel : public ChainModel
{
public:
  /**
   *  \brief Create a new camera 3d model (Kinect/Primesense).
   *  \param model The KDL model of the robot's kinematics.
   *  \param root The name of the root link, must be consistent across all
   *         models used for error modeling. Usually 'base_link'.
   *  \param tip The tip of the chain.
   */
  Camera3dModel(const std::string& name, KDL::Tree model, std::string root, std::string tip);
  virtual ~Camera3dModel() {}

  /**
   *  \brief Compute the updated positions of the observed points
   */
  virtual std::vector<geometry_msgs::PointStamped> project(
    const robot_calibration_msgs::CalibrationData& data,
    const CalibrationOffsetParser& offsets);
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_MODELS_CAMERA3D_H
