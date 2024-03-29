/*
 * Copyright (C) 2022 Michael Ferguson
 * Copyright (C) 2015-2016 Fetch Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_CAPTURE_DEPTH_CAMERA_H
#define ROBOT_CALIBRATION_CAPTURE_DEPTH_CAMERA_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>
#include <robot_calibration_msgs/msg/extended_camera_info.hpp>

namespace robot_calibration
{

/** @brief Base class for a feature finder. */
class DepthCameraInfoManager
{
public:
  DepthCameraInfoManager() : camera_info_valid_(false) {}
  virtual ~DepthCameraInfoManager() {}

  bool init(const std::string& name, rclcpp::Node::SharedPtr node, const rclcpp::Logger& logger)
  {
    std::string topic_name =
      node->declare_parameter<std::string>(name + ".camera_info_topic", "/head_camera/depth/camera_info");
    camera_info_subscriber_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic_name, 1, std::bind(&DepthCameraInfoManager::cameraInfoCallback, this, std::placeholders::_1));

    // Set default driver parameters
    z_offset_mm_ = 0;
    z_scaling_ = 1.0;

    // Get parameters of drivers
    std::string driver_name =
      node->declare_parameter<std::string>(name + ".camera_driver", "/head_camera/driver");
    auto params_client = std::make_shared<rclcpp::SyncParametersClient>(node, driver_name);
    if (params_client->wait_for_service(std::chrono::seconds(10)))
    {
      auto parameters = params_client->get_parameters({"z_offset_mm", "z_scaling"});
      for (auto& param : parameters)
      {
        if (param.get_name() == "z_offset_mm")
        {
          z_offset_mm_ = param.as_int();
          RCLCPP_INFO(node->get_logger(), "Got value of %f for z_offset_mm", z_offset_mm_);
        }
        else if (param.get_name() == "z_scaling")
        {
          z_scaling_ = param.as_double();
          RCLCPP_INFO(node->get_logger(), "Got value of %f for z_scaling", z_scaling_);
        }
      }
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Unable to get parameters from %s", driver_name.c_str());
    }

    // Wait for camera_info
    int count = 25;
    while (--count)
    {
      if (camera_info_valid_)
      {
        return true;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(node);
    }

    RCLCPP_WARN(logger, "CameraInfo receive timed out.");
    return false;
  }

  robot_calibration_msgs::msg::ExtendedCameraInfo getDepthCameraInfo()
  {
    robot_calibration_msgs::msg::ExtendedCameraInfo info;
    info.camera_info = *camera_info_ptr_;
    info.parameters.resize(2);
    info.parameters[0].name = "z_offset_mm";
    info.parameters[0].value = z_offset_mm_;
    info.parameters[1].name = "z_scaling";
    info.parameters[1].value = z_scaling_;
    return info;
  }

private:
  void cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
  {
    camera_info_ptr_ = camera_info;
    camera_info_valid_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  bool camera_info_valid_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_ptr_;

  double z_offset_mm_;
  double z_scaling_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_DEPTH_CAMERA_H
