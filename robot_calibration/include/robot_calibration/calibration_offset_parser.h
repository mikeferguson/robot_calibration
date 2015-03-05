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

#ifndef ROBOT_CALIBRATION_CALIBRATION_OFFSET_PARSER_H
#define ROBOT_CALIBRATION_CALIBRATION_OFFSET_PARSER_H

#include <kdl/chain.hpp>

namespace robot_calibration
{

/**
 *  \brief Combined parser and configuration for calibration offsets.
 *         Holds the configuration of what is to be calibrated, and
 *         and parses the actual adjustments from the free parameters.
 */
class CalibrationOffsetParser
{
public:
  CalibrationOffsetParser();
  virtual ~CalibrationOffsetParser() {}

  /**
   *  \brief Tell the parser we wish to calibrate an active joint or other
   *         single parameter.
   *  \param joint_name The name of the joint, e.g. "shoulder_pan_joint"
   */
  bool add(const std::string name);

  /**
   *  \brief Tell the parser we wish to calibrate a fixed joint.
   *  \param name The name of the fixed joint, e.g. "head_camera_rgb_joint"
   */
  bool addFrame(const std::string name,
                bool calibrate_x, bool calibrate_y, bool calibrate_z,
                bool calibrate_roll, bool calibrate_pitch, bool calibrate_yaw);

  /** \brief Update the offsets based on free_params from ceres-solver. */
  bool update(const double* const free_params);

  /** \brief Get the offset. */
  double get(const std::string name) const;

  /**
   *  \brief Get the offset for a frame calibration
   *  \param name The name of the fixed joint, e.g. "head_camera_rgb_joint"
   *  \param offset The KDL::Frame to fill in the offset.
   *  \returns True if there is an offset to apply, false if otherwise.
   */
  bool getFrame(const std::string name, KDL::Frame& offset) const;

  /** \returns The number of free parameters being parsed */
  int size();

  /** \brief Get all the current offsets as a YAML */
  std::string getOffsetYAML();

  /** \brief Update the urdf with the new offsets */
  std::string updateURDF(const std::string& urdf);

private:
  // Names of parameters being calibrated. The order of this vector
  // is the same as the free_param order will be interpreted.
  std::vector<std::string> parameter_names_;

  // Names of frames being calibrated.
  std::vector<std::string> frame_names_;

  // Values of parameters from last update
  std::vector<double> parameter_offsets_;

  // No copy
  CalibrationOffsetParser(const CalibrationOffsetParser&);
  CalibrationOffsetParser& operator=(const CalibrationOffsetParser&);
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CALIBRATION_OFFSET_PARSER_H