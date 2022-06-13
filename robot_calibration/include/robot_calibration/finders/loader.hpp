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

#ifndef ROBOT_CALIBRATION_FINDERS_LOADER_HPP
#define ROBOT_CALIBRATION_FINDERS_LOADER_HPP

#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <robot_calibration/finders/feature_finder.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robot_calibration
{

typedef std::shared_ptr<FeatureFinder> FeatureFinderPtr;
typedef std::map<std::string, FeatureFinderPtr > FeatureFinderMap;

/**
 * @brief Load feature finders, based on param server config.
 */
class FeatureFinderLoader
{
public:
  FeatureFinderLoader() :
    plugin_loader_("robot_calibration", "robot_calibration::FeatureFinder")
  {
  }

  bool load(rclcpp::Node::SharedPtr node,
            FeatureFinderMap& features)
  {
    // Setup tf2 interface
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Empty the mapping
    features.clear();

    auto logger = node->get_logger();

    // Construct finders to detect relevant features
    std::vector<std::string> feature_names =
      node->declare_parameter<std::vector<std::string>>("features", std::vector<std::string>());
    if (feature_names.empty())
    {
      RCLCPP_FATAL(logger, "Parameter 'features' is not set!");
      return false;
    }

    // Load each finder
    RCLCPP_INFO(logger, "Loading %d feature finders.", static_cast<int>(feature_names.size()));
    for (auto name : feature_names)
    {
      // Get finder type
      std::string type =
        node->declare_parameter<std::string>(name + ".type", std::string());
      
      if (type == "")
      {
        RCLCPP_FATAL(logger, "Feature finder %s has no type defined", name.c_str());
        return false;
      }

      // Load correct finder
      FeatureFinderPtr finder;
      RCLCPP_INFO(logger, "  New %s: %s", type.c_str(), name.c_str());
      finder = plugin_loader_.createSharedInstance(type);
      if (finder && finder->init(name, tf2_buffer_, node))
      {
        features[name] = finder;
      }
    }

    return true;
  }

private:
  pluginlib::ClassLoader<robot_calibration::FeatureFinder> plugin_loader_;

  // Shared TF2 buffer (since listener creates an extra rclcpp::Node)
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_FINDERS_LOADER_HPP
