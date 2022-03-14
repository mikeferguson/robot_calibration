/*
 * Copyright (C) 2018-2020 Michael Ferguson
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

//#include <fstream>

#include <ros/ros.h>
#include <robot_calibration/load_bag.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/ceres/optimization_params.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>

int main(int argc, char** argv)
{
  // What bag to visualize
  if (argc == 1)
  {
    std::cerr << std::endl;
    std::cerr << "usage:" << std::endl;
    std::cerr << "  viz calibration_data.bag [offsets.yaml]" << std::endl;
    std::cerr << std::endl;
    return -1;
  }
  std::string bag_name = argv[1];

  // Start up ROS
  ros::init(argc, argv, "robot_calibration_viz");
  ros::NodeHandle nh("~");

  // Publisher of fake joint states (latched)
  ros::Publisher state = nh.advertise<sensor_msgs::JointState>("/fake_controller_joint_states", 1, true);

  // Publisher of visualization (latched)
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("data", 10, true);

  // The calibration data
  std_msgs::String description_msg;
  std::vector<robot_calibration_msgs::CalibrationData> data;
  if (!robot_calibration::load_bag(bag_name, description_msg, data))
  {
    // Error will be printed in function
    return -1;
  }

  // Load KDL from URDF
  urdf::Model model;
  KDL::Tree tree;
  if (!model.initString(description_msg.data))
  {
    ROS_FATAL("Failed to parse URDF.");
    return -1;
  }
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_FATAL("Failed to construct KDL tree");
    return -1;
  }

  // Get parameters
  robot_calibration::OptimizationParams params;
  robot_calibration::CalibrationOffsetParser offsets;
  params.LoadFromROS(nh);
  ROS_INFO_STREAM("Publishing markers in " << params.base_link << " frame.");

  // Create models for reprojection
  std::map<std::string, robot_calibration::ChainModel*> models;
  std::map<std::string, ros::Publisher> camera_pubs;
  std::vector<std::string> model_names;
  for (size_t i = 0; i < params.models.size(); ++i)
  {
    if (params.models[i].type == "chain")
    {
      ROS_INFO_STREAM("Creating chain '" << params.models[i].name << "' from " <<
                                            params.base_link << " to " <<
                                            params.models[i].params["frame"]);
      robot_calibration::ChainModel* model = new robot_calibration::ChainModel(params.models[i].name, tree, params.base_link, params.models[i].params["frame"]);
      models[params.models[i].name] = model;
      model_names.push_back(params.models[i].name);
    }
    else if (params.models[i].type == "camera3d")
    {
      ROS_INFO_STREAM("Creating camera3d '" << params.models[i].name << "' in frame " <<
                                               params.models[i].params["frame"]);
      robot_calibration::Camera3dModel* model = new robot_calibration::Camera3dModel(params.models[i].name, tree, params.base_link, params.models[i].params["frame"]);
      models[params.models[i].name] = model;
      model_names.push_back(params.models[i].name);
      camera_pubs[params.models[i].name] = nh.advertise<sensor_msgs::PointCloud2>(params.models[i].name, 1);
    }
    else
    {
      // ERROR unknown
    }
  }

  // Setup our colors
  std::vector<std_msgs::ColorRGBA> model_colors;
  {
    // Index 0 is white -- used for first points
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.b = 1.0;
    color.g = 1.0;
    color.a = 1.0;
    model_colors.push_back(color);
    while (model_colors.size() < model_names.size() + 1)
    {
      // Red
      std_msgs::ColorRGBA color;
      color.r = 1;
      color.g = 0;
      color.b = 0;
      color.a = 1;
      model_colors.push_back(color);
      // Green
      color.r = 0;
      color.g = 1;
      color.b = 0;
      color.a = 1;
      model_colors.push_back(color);
      // Green
      color.r = 0;
      color.g = 0;
      color.b = 1;
      color.a = 1;
      model_colors.push_back(color);
    }
  }

  // Load initial values for offsets (if any)
  for (size_t i = 0; i < params.free_params.size(); ++i)
  {
    offsets.add(params.free_params[i]);
  }
  for (size_t i = 0; i < params.free_frames.size(); ++i)
  {
    offsets.addFrame(params.free_frames[i].name,
                     params.free_frames[i].x,
                     params.free_frames[i].y,
                     params.free_frames[i].z,
                     params.free_frames[i].roll,
                     params.free_frames[i].pitch,
                     params.free_frames[i].yaw);
  }
  for (size_t i = 0; i < params.free_frames_initial_values.size(); ++i)
  {
    if (!offsets.setFrame(params.free_frames_initial_values[i].name,
                          params.free_frames_initial_values[i].x,
                          params.free_frames_initial_values[i].y,
                          params.free_frames_initial_values[i].z,
                          params.free_frames_initial_values[i].roll,
                          params.free_frames_initial_values[i].pitch,
                          params.free_frames_initial_values[i].yaw))
    {
      ROS_ERROR_STREAM("Error setting initial value for " <<
                       params.free_frames_initial_values[i].name);
    }
  }

  // Load final values for offsets (if requested)
  if (argc == 3)
  {
    std::string offsets_yaml = argv[2];
    if (!offsets.loadOffsetYAML(offsets_yaml))
    {
      ROS_FATAL("Unable to load offsets from YAML");
      return -1;
    }
  }

  // Publish messages
  for (size_t i = 0; i < data.size(); ++i)
  {
    // Break out if ROS is dead
    if (!ros::ok())
      break;

    // Publish a marker array
    visualization_msgs::MarkerArray markers;
    for (size_t m = 0; m < model_names.size(); ++m)
    {
      // Project through model
      std::vector<geometry_msgs::PointStamped> points;
      points = models[model_names[m]]->project(data[i], offsets);

      if (points.empty())
      {
        continue;
      }

      // Convert into marker
      visualization_msgs::Marker msg;
      msg.header.frame_id = params.base_link;
      msg.header.stamp = ros::Time::now();
      msg.ns = model_names[m];
      msg.id = m;
      msg.type = msg.SPHERE_LIST;
      msg.pose.orientation.w = 1.0;
      msg.scale.x = 0.005;
      msg.scale.y = 0.005;
      msg.scale.z = 0.005;
      msg.points.push_back(points[0].point);
      msg.colors.push_back(model_colors[0]);
      for (size_t p = 1; p < points.size(); ++p)
      {
        msg.points.push_back(points[p].point);
        msg.colors.push_back(model_colors[m+1]);
      }
      markers.markers.push_back(msg);
    }
    pub.publish(markers);

    // Publish the joint states
    sensor_msgs::JointState state_msg = data[i].joint_states;
    for (size_t j = 0; j < state_msg.name.size(); ++j)
    {
      double offset = offsets.get(state_msg.name[j]);
      state_msg.position[j] += offset;
    }
    state.publish(state_msg);

    // Publish sensor data (if present)
    for (size_t obs = 0; obs < data[i].observations.size(); ++obs)
    {
      if (data[i].observations[obs].cloud.height != 0)
      {
        auto pub = camera_pubs.find(data[i].observations[obs].sensor_name);
        if (pub != camera_pubs.end())
        {
          pub->second.publish(data[i].observations[obs].cloud);
        }
      }
    }

    // Wait to proceed
    std::cout << "Press enter to continue...";
    std::string throwaway;
    std::getline(std::cin, throwaway);
  }

  return 0;
}