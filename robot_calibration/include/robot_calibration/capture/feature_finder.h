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

#ifndef ROBOT_CALIBRATION_CAPTURE_FEATURE_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_FEATURE_FINDER_H

#include <map>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

/**
 *  @brief Base class for a feature finder.
 */
class FeatureFinder
{
public:
  FeatureFinder(ros::NodeHandle & n) {};
  virtual ~FeatureFinder() {};

  virtual bool find(robot_calibration_msgs::CalibrationData * msg) = 0;
};

typedef boost::shared_ptr<FeatureFinder> FeatureFinderPtr;
typedef std::map<std::string, FeatureFinderPtr > FeatureFinderMap;

/**
 * @brief Load feature finders, based on param server config.
 */
bool loadFeatureFinders(ros::NodeHandle& nh,
                        FeatureFinderMap& features);

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_FEATURE_FINDER_H
