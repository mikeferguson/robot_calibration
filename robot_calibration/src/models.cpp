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

#include <robot_calibration/models/chain.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/camera2d.h>

namespace robot_calibration
{

double positionFromMsg(const std::string& name,
    const sensor_msgs::JointState& msg)
{
  for (size_t i = 0; i < msg.name.size(); ++i)
  {
    if (msg.name[i] == name)
      return msg.position[i];
  }

  std::cerr << "Unable to find " << name << " in sensor_msgs::JointState" << std::endl;

  return 0.0;
}

ChainModel::ChainModel(const std::string& name, KDL::Tree model, std::string root, std::string tip) :
  root_(root), tip_(tip), name_(name)
{
  // Create a KDL::Chain
  if (!model.getChain(root, tip, chain_))
    std::cerr << "Failed to get chain" << std::endl;
  if (!model.getChain(root, "head_camera_rgb_optical_frame", chaincam_))
    std::cerr << "Failed to get chain" << std::endl;

}

std::vector<geometry_msgs::PointStamped> ChainModel::project(
    const robot_calibration_msgs::CalibrationData& data,
    const CalibrationOffsetParser& offsets)
{
  std::vector<geometry_msgs::PointStamped> points;

  // Determine which observation to use
  int sensor_idx = -1;
  for (size_t obs = 0; obs < data.observations.size(); obs++)
  {
    if (data.observations[obs].sensor_name == name_)
    {
      sensor_idx = obs;
//      std::cout << "sensor_idx" << sensor_idx <<  std::endl;
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

  KDL::Frame fk = getChainFK(offsets, data.joint_states);

  for (size_t i = 0; i < points.size(); ++i)
  {
    points[i].header.frame_id = root_;  // fk returns point in root_ frame

    KDL::Frame p(KDL::Frame::Identity());
    p.p.x(data.observations[sensor_idx].features[i].point.x);
    p.p.y(data.observations[sensor_idx].features[i].point.y);
    p.p.z(data.observations[sensor_idx].features[i].point.z);

    if (data.observations[sensor_idx].features[i].header.frame_id != tip_)
    {
      KDL::Frame p2(KDL::Frame::Identity());
      if (offsets.getFrame(data.observations[sensor_idx].features[i].header.frame_id, p2))
      {
        p = p2 * p;
      }
    }

    p = fk * p;

    points[i].point.x = p.p.x();
    points[i].point.y = p.p.y();
    points[i].point.z = p.p.z();
  }
  return points;
}

KDL::Frame ChainModel::getChainFK(const CalibrationOffsetParser& offsets,
    const sensor_msgs::JointState& state)
{
  // FK from root to tip
  KDL::Frame p_out = KDL::Frame::Identity();

  // Step through joints
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
  {
    std::string name = chain_.getSegment(i).getJoint().getName();
    KDL::Frame correction;

    // Apply any frame calibration
    if (offsets.getFrame(name, correction))
      p_out = p_out * correction;

    // Apply any joint offset calibration
    if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      double p = positionFromMsg(name, state) + offsets.get(name);
      p_out = p_out * chain_.getSegment(i).pose(p);
    }
    else
    {
      p_out = p_out * chain_.getSegment(i).pose(0.0);
    }
  }
  return p_out;
}

KDL::Frame ChainModel::getChainFKcam(const CalibrationOffsetParser& offsets,
    const sensor_msgs::JointState& state)
{
  // FK from root to tip
  KDL::Frame p_out = KDL::Frame::Identity();

  // Step through joints
  for (size_t i = 0; i < chaincam_.getNrOfSegments(); ++i)
  {
    std::string name = chaincam_.getSegment(i).getJoint().getName();
    KDL::Frame correction;

    // Apply any frame calibration
    if (offsets.getFrame(name, correction))
      p_out = p_out * correction;

    // Apply any joint offset calibration
    if (chaincam_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      double p = positionFromMsg(name, state) + offsets.get(name);
      p_out = p_out * chaincam_.getSegment(i).pose(p);
    }
    else
    {
      p_out = p_out * chaincam_.getSegment(i).pose(0.0);
    }
  }
  return p_out;
}


std::vector<geometry_msgs::PointStamped> ChainModel::project_(
    const robot_calibration_msgs::CalibrationData& data,
    const CalibrationOffsetParser& offsets)
{
  std::vector<geometry_msgs::PointStamped> points;

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

  KDL::Frame fk = getChainFK(offsets, data.joint_states);

  for (size_t i = 0; i < points.size(); ++i)
  {
    points[i].header.frame_id = "base_link";

    KDL::Frame p(KDL::Frame::Identity());
    p.p.x(data.observations[sensor_idx].features[i].point.x);
    p.p.y(data.observations[sensor_idx].features[i].point.y);
    p.p.z(data.observations[sensor_idx].features[i].point.z);

    if (data.observations[sensor_idx].features[i].header.frame_id != tip_)
    {
      std::cout << "tip" << tip_ << std::endl;
      std::cout << data.observations[sensor_idx].features[i].header.frame_id << std::endl;
      KDL::Frame p2(KDL::Frame::Identity());
      if (offsets.getFrame(data.observations[sensor_idx].features[i].header.frame_id, p2))
      {
        p = p2 * p;
      }
    } 

    p = fk * p;

    points[i].point.x = p.p.x();
    points[i].point.y = p.p.y();
    points[i].point.z = p.p.z();
  }

  KDL::Frame fk1 = getChainFKcam(offsets, data.joint_states);

  for (size_t i = 0; i < points.size(); ++i)
  {
    KDL::Frame p(KDL::Frame::Identity());
    p.p.x(points[i].point.x);
    p.p.y(points[i].point.y);
    p.p.z(points[i].point.z);

    fk = fk1.Inverse();
    p = fk * p;

/*    KDL::Frame p1(KDL::Frame::Identity());

    if(points[i].header.frame_id != "head_camera_rgb_optical_frame")
    { 
      KDL::Frame p2(KDL::Frame::Identity());
      if (offsets.getFrame("head_camera_rgb_joint", p2))
      {
        p1 = p2 * p1;
      }
    }

    p1 = p1.Inverse();
    p = p1 * p;*/
    points[i].header.frame_id = "head_camera_rgb_optical_frame";
    points[i].point.x = p.p.x();
    points[i].point.y = p.p.y();
    points[i].point.z = p.p.z();
  }

  return points;
}


Camera3dModel::Camera3dModel(const std::string& name, KDL::Tree model, std::string root, std::string tip) :
  ChainModel(name, model, root, tip)
{
  // TODO add additional parameters for unprojecting observations using initial parameters
}

std::vector<geometry_msgs::PointStamped> Camera3dModel::project(
    const robot_calibration_msgs::CalibrationData& data,
    const CalibrationOffsetParser& offsets)
{
  std::vector<geometry_msgs::PointStamped> points;

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
  if (data.observations[sensor_idx].ext_camera_info.camera_info.P.size() != 12)
    std::cerr << "Unexpected CameraInfo projection matrix size" << std::endl;

  double camera_fx = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_FX_INDEX];
  double camera_fy = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_FY_INDEX];
  double camera_cx = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_CX_INDEX];
  double camera_cy = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_CY_INDEX];

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
  double new_camera_fx = camera_fx * (1.0 + offsets.get(name_+"_fx"));
  double new_camera_fy = camera_fy * (1.0 + offsets.get(name_+"_fy"));
  double new_camera_cx = camera_cx * (1.0 + offsets.get(name_+"_cx"));
  double new_camera_cy = camera_cy * (1.0 + offsets.get(name_+"_cy"));
  double new_z_offset = offsets.get(name_+"_z_offset");
  double new_z_scaling = 1.0 + offsets.get(name_+"_z_scaling");
 // std::cout << "name" << name_ << std::endl;
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
    //std::cout << "before " << "\t" << x << "\t" << y << "\t" << z << std::endl;
    KDL::Frame pt(KDL::Frame::Identity());

    // Reproject through new calibrated parameters
    pt.p.z((depth + new_z_offset) * new_z_scaling);
    pt.p.x((u - new_camera_cx) * pt.p.z() / new_camera_fx);
    pt.p.y((v - new_camera_cy) * pt.p.z() / new_camera_fy);

    // Project through fk
    pt = fk * pt;
    //std::cout << "after " << "\t" << pt.p.x() << "\t" << pt.p.y() << "\t" << pt.p.z() << std::endl;

    points[i].point.x = pt.p.x();
    points[i].point.y = pt.p.y();
    points[i].point.z = pt.p.z();
    points[i].header.frame_id = root_;
  }

  return points;
}

Camera2dModel::Camera2dModel(const std::string& name, KDL::Tree model, std::string root, std::string tip) :
  ChainModel(name, model, root, tip)
{
  // TODO add additional parameters for unprojecting observations using initial parameters
  // 
}

std::vector<geometry_msgs::PointStamped> Camera2dModel::project_(
    const robot_calibration_msgs::CalibrationData& data,
    std::vector<geometry_msgs::PointStamped> arm_pts,
    const CalibrationOffsetParser& offsets)
{
  std::vector<geometry_msgs::PointStamped> points;

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
  if (data.observations[sensor_idx].ext_camera_info.camera_info.P.size() != 12)
    std::cerr << "Unexpected CameraInfo projection matrix size" << std::endl;

  double camera_fx = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_FX_INDEX];
  double camera_fy = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_FY_INDEX];
  double camera_cx = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_CX_INDEX];
  double camera_cy = data.observations[sensor_idx].ext_camera_info.camera_info.P[CAMERA_INFO_P_CY_INDEX];

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
      //std::cout << "z_scaling" << z_scaling << std::endl;
    }
    else if (data.observations[sensor_idx].ext_camera_info.parameters[i].name == "z_offset_mm")
    {
      z_offset = data.observations[sensor_idx].ext_camera_info.parameters[i].value / 1000.0;  // (mm -> m)
      //std::cout << "z_offset" << z_offset << std::endl;
    }
  }

  // Get calibrated camera info
  double new_camera_fx = camera_fx * (1.0 + offsets.get(name_+"_fx"));
  double new_camera_fy = camera_fy * (1.0 + offsets.get(name_+"_fy"));
  double new_camera_cx = camera_cx * (1.0 + offsets.get(name_+"_cx"));
  double new_camera_cy = camera_cy * (1.0 + offsets.get(name_+"_cy"));
  double new_z_offset = offsets.get(name_+"_z_offset");
  double new_z_scaling = 1.0 + offsets.get(name_+"_z_scaling");
  //std::cout << "name" << name_ << std::endl;
  points.resize(data.observations[sensor_idx].features.size());

  // Get position of camera frame
  KDL::Frame fk = getChainFK(offsets, data.joint_states);

  for (size_t i = 0; i < points.size(); ++i)
  {
    // TODO: warn if frame_id != tip?
    double x = arm_pts[i].point.x;
    double y = arm_pts[i].point.y;
    double z = arm_pts[i].point.z;

    //std::cout << x << "\t" << y << "\t" << z << std::endl;
    // Unproject through parameters stored at runtime
    double u = x * new_camera_fx / z + new_camera_cx;
    double v = y * new_camera_fy / z + new_camera_cy;
    double depth = z/new_z_scaling - new_z_offset;

    points[i].point.x = u;
    points[i].point.y = v;
    points[i].point.z = depth;
    points[i].header.frame_id = "head_camera_rgb_optical_frame";
  }

  return points;
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
