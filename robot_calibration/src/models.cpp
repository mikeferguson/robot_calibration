/*
 * Copyright (C) 2022 Michael Ferguson
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

#include <iostream>
#include <robot_calibration/models/chain3d.hpp>
#include <robot_calibration/models/camera3d.hpp>

namespace robot_calibration
{

double positionFromMsg(const std::string& name,
                       const sensor_msgs::msg::JointState& msg)
{
  for (size_t i = 0; i < msg.name.size(); ++i)
  {
    if (msg.name[i] == name)
      return msg.position[i];
  }

  std::cerr << "Unable to find " << name << " in sensor_msgs::JointState" << std::endl;

  return 0.0;
}

Chain3dModel::Chain3dModel(const std::string& name, KDL::Tree model, std::string root, std::string tip) :
    root_(root), tip_(tip), name_(name)
{
  // Create a KDL::Chain
  if (!model.getChain(root, tip, chain_))
  {
    auto error_msg = std::string{"Failed to build a chain model from "} + root + " to " + tip + ", check the link names";
    //ROS_ERROR("%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }
}

std::vector<geometry_msgs::msg::PointStamped> Chain3dModel::project(
    const robot_calibration_msgs::msg::CalibrationData& data,
    const CalibrationOffsetParser& offsets)
{
  // Projected points, to be returned
  std::vector<geometry_msgs::msg::PointStamped> points;

  // Determine which observation to use
  int sensor_idx = -1;
  for (size_t obs = 0; obs < data.observations.size(); obs++)
  {
    if (data.observations[obs].sensor_name == name_)
    {
      sensor_idx = obs;
      break;
    }
  }

  if (sensor_idx < 0)
  {
    // TODO: any sort of error message?
    return points;
  }

  // Resize to match # of features
  points.resize(data.observations[sensor_idx].features.size());

  // Get the projection from forward kinematics of the robot chain
  KDL::Frame fk = getChainFK(offsets, data.joint_states);

  // Project each individual point
  for (size_t i = 0; i < points.size(); ++i)
  {
    points[i].header.frame_id = root_;  // fk returns point in root_ frame

    KDL::Frame p(KDL::Frame::Identity());
    p.p.x(data.observations[sensor_idx].features[i].point.x);
    p.p.y(data.observations[sensor_idx].features[i].point.y);
    p.p.z(data.observations[sensor_idx].features[i].point.z);

    // This is primarily for the case of checkerboards
    //   The observation is in "checkerboard" frame, but the tip of the
    //   kinematic chain is typically something like "wrist_roll_link".
    if (data.observations[sensor_idx].features[i].header.frame_id != tip_)
    {
      KDL::Frame p2(KDL::Frame::Identity());
      if (offsets.getFrame(data.observations[sensor_idx].features[i].header.frame_id, p2))
      {
        // We have to apply the frame offset before the FK projection
        p = p2 * p;
      }
    }

    // Apply the FK projection
    p = fk * p;

    points[i].point.x = p.p.x();
    points[i].point.y = p.p.y();
    points[i].point.z = p.p.z();
  }

  return points;
}

KDL::Frame Chain3dModel::getChainFK(const CalibrationOffsetParser& offsets,
                                  const sensor_msgs::msg::JointState& state)
{
  // FK from root to tip
  KDL::Frame p_out = KDL::Frame::Identity();

  // Step through joints
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
  {
    std::string name = chain_.getSegment(i).getJoint().getName();
    KDL::Frame correction = KDL::Frame::Identity();
    offsets.getFrame(name, correction);

    KDL::Frame pose;
    if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      // Apply any joint offset calibration
      double p = positionFromMsg(name, state) + offsets.get(name);
      pose = chain_.getSegment(i).pose(p);
    }
    else
    {
      pose = chain_.getSegment(i).pose(0.0);
    }

    KDL::Frame totip = chain_.getSegment(i).getFrameToTip();

    // Apply any frame calibration on the joint <origin> frame
    p_out = p_out * KDL::Frame(pose.p + totip.M * correction.p);
    p_out = p_out * KDL::Frame(totip.M * correction.M * totip.M.Inverse() * pose.M);

  }
  return p_out;
}

std::string Chain3dModel::getName() const
{
  return name_;
}

std::string Chain3dModel::getType() const
{
  return "Chain3dModel";
}

Camera3dModel::Camera3dModel(const std::string& name, const std::string& param_name, KDL::Tree model, std::string root, std::string tip) :
    Chain3dModel(name, model, root, tip),
    param_name_(param_name)
{
  // TODO add additional parameters for unprojecting observations using initial parameters
}

std::vector<geometry_msgs::msg::PointStamped> Camera3dModel::project(
    const robot_calibration_msgs::msg::CalibrationData& data,
    const CalibrationOffsetParser& offsets)
{
  std::vector<geometry_msgs::msg::PointStamped> points;

  // Determine which observation to use
  int sensor_idx = -1;
  for (size_t obs = 0; obs < data.observations.size(); obs++)
  {
    if (data.observations[obs].sensor_name == name_)
    {
      sensor_idx = obs;
      break;
    }
  }

  if (sensor_idx < 0)
  {
    // TODO: any sort of error message?
    return points;
  }

  // Get existing camera info
  if (data.observations[sensor_idx].ext_camera_info.camera_info.p.size() != 12)
    std::cerr << "Unexpected CameraInfo projection matrix size" << std::endl;

  double camera_fx = data.observations[sensor_idx].ext_camera_info.camera_info.p[CAMERA_INFO_P_FX_INDEX];
  double camera_fy = data.observations[sensor_idx].ext_camera_info.camera_info.p[CAMERA_INFO_P_FY_INDEX];
  double camera_cx = data.observations[sensor_idx].ext_camera_info.camera_info.p[CAMERA_INFO_P_CX_INDEX];
  double camera_cy = data.observations[sensor_idx].ext_camera_info.camera_info.p[CAMERA_INFO_P_CY_INDEX];

  /*
   * z_scale and z_offset defined in openni2_camera/src/openni2_driver.cpp
   * new_depth_mm = (depth_mm + z_offset_mm) * z_scale
   * NOTE: these work on integer values not floats
   */
  double z_offset = 0.0;
  double z_scaling = 1.0;
  for (size_t i = 0; i < data.observations[sensor_idx].ext_camera_info.parameters.size(); i++)
  {
    if (data.observations[sensor_idx].ext_camera_info.parameters[i].name == "z_scaling")
    {
      z_scaling = data.observations[sensor_idx].ext_camera_info.parameters[i].value;
    }
    else if (data.observations[sensor_idx].ext_camera_info.parameters[i].name == "z_offset_mm")
    {
      z_offset = data.observations[sensor_idx].ext_camera_info.parameters[i].value / 1000.0;  // (mm -> m)
    }
  }

  // Get calibrated camera info
  double new_camera_fx = camera_fx * (1.0 + offsets.get(param_name_ + "_fx"));
  double new_camera_fy = camera_fy * (1.0 + offsets.get(param_name_ + "_fy"));
  double new_camera_cx = camera_cx * (1.0 + offsets.get(param_name_ + "_cx"));
  double new_camera_cy = camera_cy * (1.0 + offsets.get(param_name_ + "_cy"));
  double new_z_offset = offsets.get(param_name_ + "_z_offset");
  double new_z_scaling = 1.0 + offsets.get(param_name_ + "_z_scaling");

  points.resize(data.observations[sensor_idx].features.size());

  // Get position of camera frame
  KDL::Frame fk = getChainFK(offsets, data.joint_states);

  for (size_t i = 0; i < points.size(); ++i)
  {
    // TODO: warn if frame_id != tip?
    double x = data.observations[sensor_idx].features[i].point.x;
    double y = data.observations[sensor_idx].features[i].point.y;
    double z = data.observations[sensor_idx].features[i].point.z;

    // Unproject through parameters stored at runtime
    double u = x * camera_fx / z + camera_cx;
    double v = y * camera_fy / z + camera_cy;
    double depth = z/z_scaling - z_offset;

    KDL::Frame pt(KDL::Frame::Identity());

    // Reproject through new calibrated parameters
    pt.p.z((depth + new_z_offset) * new_z_scaling);
    pt.p.x((u - new_camera_cx) * pt.p.z() / new_camera_fx);
    pt.p.y((v - new_camera_cy) * pt.p.z() / new_camera_fy);

    // Project through fk
    pt = fk * pt;

    points[i].point.x = pt.p.x();
    points[i].point.y = pt.p.y();
    points[i].point.z = pt.p.z();
    points[i].header.frame_id = root_;
  }

  return points;
}

std::string Camera3dModel::getType() const
{
  return "Camera3dModel";
}

KDL::Rotation rotation_from_axis_magnitude(const double x, const double y, const double z)
{
  double magnitude = sqrt(x*x + y*y + z*z);

  if (magnitude == 0.0)
    return KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  return KDL::Rotation::Quaternion(x/magnitude * sin(magnitude/2.0),
                                   y/magnitude * sin(magnitude/2.0),
                                   z/magnitude * sin(magnitude/2.0),
                                   cos(magnitude/2.0));
}

void axis_magnitude_from_rotation(const KDL::Rotation& r, double& x, double& y, double& z)
{
  double qx, qy, qz, qw;
  r.GetQuaternion(qx, qy, qz, qw);

  if (qw == 1.0)
  {
    x = y = z = 0.0;
    return;
  }

  double magnitude = 2 * acos(qw);
  double k = sqrt(1 - (qw*qw));

  x = (qx / k) * magnitude;
  y = (qy / k) * magnitude;
  z = (qz / k) * magnitude;
}

}  // namespace robot_calibration
