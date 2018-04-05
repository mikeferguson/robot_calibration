/*
 * Copyright (C) 2018 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_FEATURE_FINDER_LOADER_H
#define ROBOT_CALIBRATION_FEATURE_FINDER_LOADER_H

#include <map>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <robot_calibration/plugins/feature_finder.h>

namespace robot_calibration
{

typedef boost::shared_ptr<FeatureFinder> FeatureFinderPtr;
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

  bool load(ros::NodeHandle& nh,
            FeatureFinderMap& features)
  {
    // Empty the mapping
    features.clear();

    // Construct finders to detect relevant features
    XmlRpc::XmlRpcValue finder_params;
    if (!nh.getParam("features", finder_params))
    {
      ROS_FATAL("Parameter 'features' is not set!");
      return false;
    }

    // Should be a struct (mapping name -> config)
    if (finder_params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_FATAL("Parameter 'features' should be a struct.");
      return false;
    }

    // Load each finder
    ROS_INFO("Loading %d feature finders.", (int)finder_params.size());
    for (XmlRpc::XmlRpcValue::iterator it = finder_params.begin();
         it != finder_params.end();
         it++)
    {
      // Get name(space) of this finder
      std::string name = static_cast<std::string>(it->first);
      ros::NodeHandle finder_handle(nh, "features/"+name);

      // Get finder type
      std::string type;
      if (!finder_handle.getParam("type", type))
      {
        ROS_FATAL("Feature finder %s has no type defined", name.c_str());
        return false;
      }

      // Load correct finder
      FeatureFinderPtr finder;
      ROS_INFO("  New %s: %s", type.c_str(), name.c_str());
      finder = plugin_loader_.createInstance(type);
      if (finder && finder->init(name, finder_handle))
        features[name] = finder;
    }

    return true;
  }
private:
  pluginlib::ClassLoader<robot_calibration::FeatureFinder> plugin_loader_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_FEATURE_FINDER_LOADER_H
