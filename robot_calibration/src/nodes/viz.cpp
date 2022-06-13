/*
 * Copyright (C) 2018-2022 Michael Ferguson
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

#include <rclcpp/rclcpp.hpp>
#include <robot_calibration/util/calibration_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <robot_calibration/optimization/offsets.hpp>
#include <robot_calibration/optimization/params.hpp>
#include <robot_calibration/models/camera3d.hpp>
#include <robot_calibration/models/chain3d.hpp>

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
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_calibration_viz");

  // Publisher of fake joint states (latched)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state =
    node->create_publisher<sensor_msgs::msg::JointState>("/fake_controller_joint_states",
      rclcpp::QoS(1).transient_local());

  // Publisher of visualization (latched)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("data",
      rclcpp::QoS(1).transient_local());

  // The calibration data
  std_msgs::msg::String description_msg;
  std::vector<robot_calibration_msgs::msg::CalibrationData> data;
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
    RCLCPP_FATAL(node->get_logger(), "Failed to parse URDF.");
    return -1;
  }
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to construct KDL tree");
    return -1;
  }

  // Load calibration steps
  std::vector<std::string> calibration_steps =
    node->declare_parameter<std::vector<std::string>>("calibration_steps", std::vector<std::string>());
  if (calibration_steps.empty())
  {
    RCLCPP_FATAL(node->get_logger(), "Parameter calibration_steps is not defined");
    return -1;
  }

  // Get parameters
  robot_calibration::OptimizationParams params;
  robot_calibration::OptimizationOffsets offsets;
  params.LoadFromROS(node, calibration_steps.front());
  RCLCPP_INFO_STREAM(node->get_logger(), "Publishing markers in " << params.base_link << " frame.");

  // Create models for reprojection
  std::map<std::string, robot_calibration::Chain3dModel*> models;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> camera_pubs;
  std::vector<std::string> model_names;
  for (size_t i = 0; i < params.models.size(); ++i)
  {
    if (params.models[i].type == "chain")
    {
      RCLCPP_INFO_STREAM(node->get_logger(),
                         "Creating chain '" << params.models[i].name << "' from " <<
                                               params.base_link << " to " <<
                                               ""); // TODO params.models[i].params["frame"]);
      robot_calibration::Chain3dModel* model = new robot_calibration::Chain3dModel(params.models[i].name, tree, params.base_link, ""); // TODO params.models[i].params["frame"]);
      models[params.models[i].name] = model;
      model_names.push_back(params.models[i].name);
    }
    else if (params.models[i].type == "camera3d")
    {
      RCLCPP_INFO_STREAM(node->get_logger(),
                         "Creating camera3d '" << params.models[i].name << "' in frame " <<
                                                  ""); // TODO params.models[i].params["frame"]);
      std::string param_name = ""; // TODO params.models[i].params["param_name"];
      if (param_name == "")
      {
        // Default to same name as sensor
        param_name = params.models[i].name;
      }
      robot_calibration::Camera3dModel* model = new robot_calibration::Camera3dModel(params.models[i].name, param_name, tree, params.base_link, ""); // TODO params.models[i].params["frame"]);
      models[params.models[i].name] = model;
      model_names.push_back(params.models[i].name);
      camera_pubs[params.models[i].name] = node->create_publisher<sensor_msgs::msg::PointCloud2>(params.models[i].name, 1);
    }
    else
    {
      // ERROR unknown
    }
  }

  // Setup our colors
  std::vector<std_msgs::msg::ColorRGBA> model_colors;
  {
    // Index 0 is white -- used for first points
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.b = 1.0;
    color.g = 1.0;
    color.a = 1.0;
    model_colors.push_back(color);
    while (model_colors.size() < model_names.size() + 1)
    {
      // Red
      std_msgs::msg::ColorRGBA color;
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
      RCLCPP_ERROR_STREAM(node->get_logger(), "Error setting initial value for " <<
                          params.free_frames_initial_values[i].name);
    }
  }

  // Load final values for offsets (if requested)
  if (argc == 3)
  {
    std::string offsets_yaml = argv[2];
    if (!offsets.loadOffsetYAML(offsets_yaml))
    {
      RCLCPP_FATAL(node->get_logger(), "Unable to load offsets from YAML");
      return -1;
    }
  }

  // Publish messages
  for (size_t i = 0; i < data.size(); ++i)
  {
    // Break out if ROS is dead
    if (!rclcpp::ok())
      break;

    // Publish a marker array
    visualization_msgs::msg::MarkerArray markers;
    for (size_t m = 0; m < model_names.size(); ++m)
    {
      // Project through model
      std::vector<geometry_msgs::msg::PointStamped> points;
      points = models[model_names[m]]->project(data[i], offsets);

      if (points.empty())
      {
        continue;
      }

      // Convert into marker
      visualization_msgs::msg::Marker msg;
      msg.header.frame_id = params.base_link;
      msg.header.stamp = node->now();
      msg.ns = model_names[m];
      msg.id = m;
      msg.type = msg.SPHERE_LIST;
      msg.pose.orientation.w = 1.0;
      msg.scale.x = 0.01;
      msg.scale.y = 0.01;
      msg.scale.z = 0.01;
      msg.points.push_back(points[0].point);
      msg.colors.push_back(model_colors[0]);
      for (size_t p = 1; p < points.size(); ++p)
      {
        msg.points.push_back(points[p].point);
        msg.colors.push_back(model_colors[m+1]);
      }
      markers.markers.push_back(msg);
    }
    pub->publish(markers);

    // Publish the joint states
    sensor_msgs::msg::JointState state_msg = data[i].joint_states;
    for (size_t j = 0; j < state_msg.name.size(); ++j)
    {
      double offset = offsets.get(state_msg.name[j]);
      state_msg.position[j] += offset;
    }
    state->publish(state_msg);

    // Publish sensor data (if present)
    for (size_t obs = 0; obs < data[i].observations.size(); ++obs)
    {
      if (data[i].observations[obs].cloud.height != 0)
      {
        auto pub = camera_pubs.find(data[i].observations[obs].sensor_name);
        if (pub != camera_pubs.end())
        {
          pub->second->publish(data[i].observations[obs].cloud);
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
