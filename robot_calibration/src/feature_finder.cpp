/*
 * Copyright (C) 2015 Fetch Robotics Inc.
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

#include <robot_calibration/capture/feature_finder.h>
#include <robot_calibration/capture/led_finder.h>
#include <robot_calibration/capture/checkerboard_finder.h>
#include <robot_calibration/capture/ground_plane_finder.h>
#include <robot_calibration/capture/gripper_depth_finder.h>
#include <robot_calibration/capture/gripper_color_finder.h>

namespace robot_calibration
{

bool loadFeatureFinders(ros::NodeHandle& nh,
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

  ROS_INFO("Loading %d feature finders.", (int)finder_params.size());

  // Load each finder
  for (XmlRpc::XmlRpcValue::iterator it = finder_params.begin();
       it != finder_params.end();
       it++)
  {
    // Get name(space) of this finder
    std::string name = static_cast<std::string>(it->first);
    ros::NodeHandle finder_handle(nh, "features/"+name);
    //std::cout << "features/"+name << std::endl;
    // Get finder type
    std::string type;
    if (!finder_handle.getParam("type", type))
    {
      ROS_FATAL("Feature finder %s has no type defined", name.c_str());
      return false;
    }

    // Load correct finder
    // TODO: this will probably be plugin-based in the future
    FeatureFinderPtr finder;
    if (type == "robot_calibration/LedFinder")
    {
      ROS_INFO("  New robot_calibration/LedFinder: %s", name.c_str());
      finder.reset(new robot_calibration::LedFinder(finder_handle));
    }
    else if (type == "robot_calibration/GroundPlaneFinder")
    {
      ROS_INFO("  New robot_calibration/GroundPlaneFinder: %s", name.c_str());
      finder.reset(new robot_calibration::GroundPlaneFinder(finder_handle));
    }
    else if (type == "robot_calibration/CheckerboardFinder")
    {
      ROS_INFO("  New robot_calibration/CheckerboardFinder: %s", name.c_str());
      finder.reset(new robot_calibration::CheckerboardFinder(finder_handle));
    }
    else if (type == "robot_calibration/GripperDepthFinder")
    {  
      ROS_INFO("  New robot_calibration/GripperDepthFinder: %s", name.c_str());
      finder.reset(new robot_calibration::GripperDepthFinder(finder_handle));
    }
    else if (type == "robot_calibration/GripperColorFinder")
    {
      ROS_INFO("  New robot_calibration/GripperColorFinder: %s", name.c_str());
      finder.reset(new robot_calibration::GripperColorFinder(finder_handle));
    }
    else
    {
      ROS_FATAL("Unknown finder: %s", type.c_str());
      return false;
    }

    features[name] = finder;
  }

  return true;
}

}  // namespace robot_calibration
