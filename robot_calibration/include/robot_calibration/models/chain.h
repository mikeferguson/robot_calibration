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

#ifndef ROBOT_CALIBRATION_MODELS_CHAIN_H
#define ROBOT_CALIBRATION_MODELS_CHAIN_H

#include <string>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <robot_calibration/calibration_offset_parser.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <robot_calibration_msgs/CalibrationData.h>

/** \brief Calibration code lives under this namespace */
namespace robot_calibration
{

/**
 *  \brief Model of a kinematic chain. This is the basic instance where we
 *         transform the world observations into the proper root frame.
 *
 *  Each world observation is an estimated point in some frame. In the case
 *  of the four led gripper, each led has an absolutely known location in
 *  the gripper_led_frame. The ordering of the points is:
 *  <PRE>
 *         _                   _
 *        | |                 | |
 *        | |                 | |
 *        | |                 | |
 *        | |                 | |
 *        | |                 | |
 *        | |     gripper     | |
 *        |---------------------|
 *        |                     |
 *        |    0           3    |
 *        |                     |
 *        |          X <------------ gripper_led_frame
 *        |                     |
 *        |    2           1    |
 *        |                     |
 *         \___________________/
 *
 *  </PRE>
 *  When using a checkerboard, each interesection on the checkerboard is an
 *  observation point. Even though checkerboard *should* be flat, the code
 *  assumes that each checkerboard intersection has 3 degrees of translational
 *  freedom. The gripper_led_frame to checkerboard_frame transformation becomes
 *  a set of free parameters in the calibration.
 *
 *  Based on the 6x5 checkboard, the ordering of the oberservation points
 *  produced by the OpenCV chessboard finder is always the same:
 *  <PRE>
 *     ____________________________
 *    |                            |
 *    |  ###   ###   ###   ###     |
 *    |  ###   ###   ###   ###     |
 *    |  ###   ###   ###  LL##     |    11  First Observation Point
 *    |     ###   ###   ##LL  ###  |    11
 *    |     ###   ###   ###   ###  |
 *    |     ###   ###   ###   ###  |    22  Second Observation Point
 *    |  ###   ###   ###   ###     |    22
 *    |  ###   ###   ###   ###     |
 *    |  ##22  ###   ###   ###     |    LL  Last (20th) Obervation Point
 *    |    22##   ###   ###   ###  |    LL
 *    |     ###   ###   ###   ###  |
 *    |    11##   ###   ###   ###  |
 *    |  ##11  ###   ###   ###     |
 *    |  ###   ###   ###   ###     |   gripper_link
 *    |  ###   ###   ###   ###     |      X-axis (increasing row)
 *    |                            |     ^
 *    |           _____            |     |     gripper_link
 *    |          |     |           |     |  ----> Z-axis (increasing col)
 *    |          |     |
 *    |          |  0<----- gripper_link frame origin
 *    |__________|     |___________
 *               |     |
 *            ___|_____|___
 *           |             |
 *           |   Gripper   |
 *           |             |
 *  </PRE>
 *  For the ideal checkboard:
 *   - All the checkerboard points should fall on or near Y=0 plane.
 *   - The intersection points are spread appart by 25mm
 *   - The X-offset from the gripper frame to the first observation point is 100mm
 *   - The Z-offset from the gripper frame to the first observation point is -50mm
 *  The checkboard points are numberered so that the lowest Z value are the first points
 */
class ChainModel
{
public:
  /**
   *  \brief Create a new chain model.
   *  \param model The KDL model of the robot's kinematics.
   *  \param root The name of the root link, must be consistent across all
   *         models used for error modeling. Usually 'base_link'.
   *  \param tip The tip of the chain.
   */
  ChainModel(const std::string& name, KDL::Tree model, std::string root, std::string tip);
  virtual ~ChainModel() {}

  /**
   *  \brief Compute the position of the estimated points.
   *  \param data The calibration data for this observation.
   *  \param offsets The offsets that the solver wants to examine.
   */
  virtual std::vector<geometry_msgs::PointStamped> project(
    const robot_calibration_msgs::CalibrationData& data,
    const CalibrationOffsetParser& offsets);

  /**
   *  \brief Compute the forward kinematics of the chain, based on the
   *         offsets and the joint positions of the state message.
   */
  KDL::Frame getChainFK(const CalibrationOffsetParser& offsets,
                        const sensor_msgs::JointState& state);

  std::string name() const;

private:
  KDL::Chain chain_;

protected:
  std::string root_;
  std::string tip_;
  std::string name_;
};

/** \brief Converts our angle-axis-with-integrated-magnitude representation to a KDL::Rotation */
KDL::Rotation rotation_from_axis_magnitude(const double x, const double y, const double z);

/** \brief Converts from KDL::Rotation to angle-axis-with-integrated-magnitude */
void axis_magnitude_from_rotation(const KDL::Rotation& r, double& x, double& y, double& z);

inline std::string ChainModel::name() const
{
  return name_;
}

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_MODELS_CHAIN_H
