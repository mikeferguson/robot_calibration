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
#include <urdf/model.h>
#include <robot_calibration/util/mesh_loader.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

int main(int argc, char** argv)
{
  // What mesh to visualize
  if (argc == 1)
  {
    std::cerr << std::endl;
    std::cerr << "usage:" << std::endl;
    std::cerr << "  viz_mesh link_name" << std::endl;
    std::cerr << std::endl;
    return -1;
  }
  std::string link_name = argv[1];

  // Start up ROS
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_calibration_mesh_viz");

  // Setup publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub;
  pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("data", 10);

  // Get robot description
  std::string robot_description =
    node->declare_parameter<std::string>("robot_description", "");

  // Load URDF and root link name
  std::shared_ptr<urdf::Model> model;
  model = std::make_shared<urdf::Model>();
  if (!model->initString(robot_description))
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to parse URDF.");
    return -1;
  }
  std::string root_name = model->getRoot()->name;

  // Create mesh loader
  robot_calibration::MeshLoader loader(model);

  // Get mesh
  robot_calibration::MeshPtr mesh = loader.getCollisionMesh(link_name);
  if (!mesh)
  {
    RCLCPP_FATAL(node->get_logger(), "Unable to load mesh");
    return -1;
  }

  // Publish vertices as marker array
  visualization_msgs::msg::MarkerArray markers;
  for (size_t t = 0; t < mesh->triangle_count; ++t)
  {
    // Create marker
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = link_name;
    //msg.header.stamp = ros::Time::now();
    msg.ns = link_name;
    msg.id = t;
    msg.type = msg.LINE_STRIP;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 0.005;
    msg.scale.y = 0.005;
    msg.scale.z = 0.005;
    msg.color.r = 1;
    msg.color.g = 0;
    msg.color.b = 0;
    msg.color.a = 1;
    // Extract the triangle
    int v1_idx = mesh->triangles[(3 * t) + 0];
    int v2_idx = mesh->triangles[(3 * t) + 1];
    int v3_idx = mesh->triangles[(3 * t) + 2];
    geometry_msgs::msg::Point v1;
    v1.x = mesh->vertices[(3 * v1_idx) + 0];
    v1.y = mesh->vertices[(3 * v1_idx) + 1];
    v1.z = mesh->vertices[(3 * v1_idx) + 2];
    geometry_msgs::msg::Point v2;
    v2.x = mesh->vertices[(3 * v2_idx) + 0];
    v2.y = mesh->vertices[(3 * v2_idx) + 1];
    v2.z = mesh->vertices[(3 * v2_idx) + 2];
    geometry_msgs::msg::Point v3;
    v3.x = mesh->vertices[(3 * v3_idx) + 0];
    v3.y = mesh->vertices[(3 * v3_idx) + 1];
    v3.z = mesh->vertices[(3 * v3_idx) + 2];
    // Add triangle as 3 line segments v1->v2, v2->v3 and v3->v1
    msg.points.push_back(v1);
    msg.points.push_back(v2);
    msg.points.push_back(v3);
    msg.points.push_back(v1);
    // Add marker
    markers.markers.push_back(msg);
  }

  while (rclcpp::ok())
  {
    pub->publish(markers);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}