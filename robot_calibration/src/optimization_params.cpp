/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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

#include <robot_calibration/ceres/optimization_params.h>

namespace robot_calibration
{

OptimizationParams::OptimizationParams() :
  base_link("base_link")
{
}

bool OptimizationParams::LoadFromROS(ros::NodeHandle& nh)
{
  nh.param("base_link", base_link, base_link);

  if (nh.hasParam("free_params"))
  {
    free_params.clear();

    XmlRpc::XmlRpcValue names;
    nh.getParam("free_params", names);
    ROS_ASSERT(names.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < names.size(); ++i)
    {
      free_params.push_back(static_cast<std::string>(names[i]));
    }
  }

  if (nh.hasParam("free_frames"))
  {
    free_frames.clear();

    XmlRpc::XmlRpcValue free_frame_params;
    nh.getParam("free_frames", free_frame_params);
    ROS_ASSERT(free_frame_params.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < free_frame_params.size(); ++i)
    {
      FreeFrameParams params;
      params.name = static_cast<std::string>(free_frame_params[i]["name"]);
      params.x = static_cast<bool>(free_frame_params[i]["x"]);
      params.y = static_cast<bool>(free_frame_params[i]["y"]);
      params.z = static_cast<bool>(free_frame_params[i]["z"]);
      params.roll = static_cast<bool>(free_frame_params[i]["roll"]);
      params.pitch = static_cast<bool>(free_frame_params[i]["pitch"]);
      params.yaw = static_cast<bool>(free_frame_params[i]["yaw"]);
      free_frames.push_back(params);
    }
  }

  if (nh.hasParam("models"))
  {
    models.clear();

    XmlRpc::XmlRpcValue model_params;
    nh.getParam("models", model_params);
    ROS_ASSERT(model_params.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < model_params.size(); ++i)
    {
      Params params;
      params.name = static_cast<std::string>(model_params[i]["name"]);
      params.type = static_cast<std::string>(model_params[i]["type"]);
      params.params = model_params[i];
      models.push_back(params);
    }
  }

  if (nh.hasParam("error_blocks"))
  {
    error_blocks.clear();

    XmlRpc::XmlRpcValue error_params;
    nh.getParam("error_blocks", error_params);
    ROS_ASSERT(error_params.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < error_params.size(); ++i)
    {
      Params params;
      params.name = static_cast<std::string>(error_params[i]["name"]);
      params.type = static_cast<std::string>(error_params[i]["type"]);
      params.params = error_params[i];
      error_blocks.push_back(params);
    }
  }

  return true;
}

}  // namespace robot_calibration
