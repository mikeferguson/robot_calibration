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

#include <fstream>
#include <sstream>
#include <camera_calibration_parsers/parse.h>
#include "robot_calibration/calibration/export.h"

namespace robot_calibration
{
bool exportResults(Optimizer& optimizer, const std::string& initial_urdf,
                   const std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  // Generate datecode
  char datecode[80];
  {
    std::time_t t = std::time(NULL);
    std::strftime(datecode, 80, "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
  }

  // Save updated URDF
  {
    std::string s = optimizer.getOffsets()->updateURDF(initial_urdf);
    std::stringstream urdf_name;
    urdf_name << "/tmp/calibrated_" << datecode << ".urdf";
    std::ofstream file;
    file.open(urdf_name.str().c_str());
    file << s;
    file.close();
  }

  // Output camera calibration(s)
  std::vector<std::string> camera_names = optimizer.getCameraNames();
  for (auto it = camera_names.begin(); it != camera_names.end(); ++it)
  {
    // Find original CameraInfo
    sensor_msgs::CameraInfo camera_info;
    bool found_camera_info = false;
    for (auto obs = data.front().observations.begin();
         obs != data.front().observations.end(); ++obs)
    {
      if (obs->sensor_name == *it)
      {
        camera_info = obs->ext_camera_info.camera_info;
        found_camera_info = true;
        break;
      }
    }

    if (!found_camera_info)
    {
      ROS_WARN("Unable to export camera_info for %s", it->c_str());
      continue;
    }

    std::stringstream depth_name;
    depth_name << "/tmp/depth_";
    if (*it != "camera")
    {
      // We include the name if the name is not "camera" for backwards compatability
      depth_name << *it << "_";
    }
    depth_name << datecode << ".yaml";
    camera_calibration_parsers::writeCalibration(depth_name.str(), "",
        robot_calibration::updateCameraInfo(
                         optimizer.getOffsets()->get(*it + "_fx"),
                         optimizer.getOffsets()->get(*it + "_fy"),
                         optimizer.getOffsets()->get(*it + "_cx"),
                         optimizer.getOffsets()->get(*it + "_cy"),
                         camera_info));

    std::stringstream rgb_name;
    rgb_name << "/tmp/rgb_";
    if (*it != "camera")
    {
      // We include the name if the name is not "camera" for backwards compatability
      rgb_name << *it << "_";
    }
    rgb_name << datecode << ".yaml";
    camera_calibration_parsers::writeCalibration(rgb_name.str(), "",
        robot_calibration::updateCameraInfo(
                         optimizer.getOffsets()->get(*it + "_fx"),
                         optimizer.getOffsets()->get(*it + "_fy"),
                         optimizer.getOffsets()->get(*it + "_cx"),
                         optimizer.getOffsets()->get(*it + "_cy"),
                         camera_info));
  }

  // Output the calibration yaml
  {
    std::stringstream yaml_name;
    yaml_name << "/tmp/calibration_" << datecode << ".yaml";
    std::ofstream file;
    file.open(yaml_name.str().c_str());
    file << optimizer.getOffsets()->getOffsetYAML();
    file << "depth_info: depth_" << datecode << ".yaml" << std::endl;
    file << "rgb_info: rgb_" << datecode << ".yaml" << std::endl;
    file << "urdf: calibrated_" << datecode << ".urdf" << std::endl;
    file.close();
  }

  return true;
}

}  // namespace robot_calibration
