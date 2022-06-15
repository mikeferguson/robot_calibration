/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2015-2017 Fetch Robotics Inc.
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

#include <math.h>
#include <robot_calibration/finders/scan_finder.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("scan_finder");

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

ScanFinder::ScanFinder() :
  waiting_(false)
{
}

bool ScanFinder::init(const std::string& name,
                      std::shared_ptr<tf2_ros::Buffer> buffer,
                      rclcpp::Node::SharedPtr node)
{
  if (!FeatureFinder::init(name, buffer, node))
  {
    return false;
  }

  clock_ = node->get_clock();

  // We subscribe to a LaserScan
  std::string topic_name;
  topic_name = node->declare_parameter<std::string>(name + ".topic", name + "/scan");
  subscriber_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_name,
    rclcpp::QoS(1).best_effort().keep_last(1),
    std::bind(&ScanFinder::scanCallback, this, std::placeholders::_1));

  // Name of the sensor model that will be used during optimization
  laser_sensor_name_ = node->declare_parameter<std::string>("sensor_name", "laser");

  // Frame to transform point cloud into before applying limits below
  //   if specified as "none", cloud will be processed in sensor frame
  transform_frame_ = node->declare_parameter<std::string>("transform_frame", "base_link");

  // It is assumed that the laser scanner operates in the X,Y plane
  // Valid points must lie within this box, in the laser frame
  min_x_ = node->declare_parameter<double>("min_x", -2.0);
  max_x_ = node->declare_parameter<double>("max_x", 2.0);
  min_y_ = node->declare_parameter<double>("min_y", -2.0);
  max_y_ = node->declare_parameter<double>("max_y", 2.0);

  // Repeat points in the Z plane a number of times at fixed distance
  z_repeats_ = node->declare_parameter<int>("z_repeats", 10);
  z_offset_ = node->declare_parameter<double>("z_offset", 0.1);

  // Should we include debug image/cloud in observations
  output_debug_ = node->declare_parameter<bool>("debug", false);

  // Publish the observation as a PointCloud2
  publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(getName() + "_points", 10);

  return true;
}

void ScanFinder::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  if (waiting_)
  {
    scan_ = *scan;
    waiting_ = false;
  }
}

bool ScanFinder::waitForScan()
{
  // Stored as weak pointer, need to grab a real shared pointer
  auto node = node_ptr_.lock();
  if (!node)
  {
    RCLCPP_ERROR(LOGGER, "Unable to get rclcpp::Node lock");
    return false;
  }

  // Initial wait cycle so that laser scan is definitely up to date.
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(node);
  }
  RCLCPP_ERROR(LOGGER, "Failed to get scan");
  return !waiting_;
}

bool ScanFinder::find(robot_calibration_msgs::msg::CalibrationData * msg)
{
  if (!waitForScan())
  {
    RCLCPP_ERROR(LOGGER, "No laser scan data");
    return false;
  }

  // Extract the points corresponding to the line
  sensor_msgs::msg::PointCloud2 cloud;
  extractPoints(cloud);
  extractObservation(cloud, msg);

  // Report success
  return true;
}

void ScanFinder::extractPoints(sensor_msgs::msg::PointCloud2& cloud)
{
  bool do_transform = transform_frame_ != "none";

  // Reset cloud
  cloud.width = 0;
  cloud.height = 0;
  cloud.header.stamp = clock_->now();
  cloud.header.frame_id = do_transform ? transform_frame_ : scan_.header.frame_id;

  // Setup cloud to be XYZ
  sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(scan_.ranges.size() * z_repeats_);

  // Create iterator to edit cloud
  sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");

  // Filter and transform scan points
  size_t line_point_count = 0;
  for (size_t i = 0; i < scan_.ranges.size(); ++i)
  {
    // Remove any NaNs in scan
    if (!std::isfinite(scan_.ranges[i]))
    {
      continue;
    }

    // Geometry to convert from scan to cloud
    double angle = scan_.angle_min + (i * scan_.angle_increment);

    // Create point (in sensor frame)
    geometry_msgs::msg::PointStamped p;
    p.point.x = cos(angle) * scan_.ranges[i];
    p.point.y = sin(angle) * scan_.ranges[i];
    p.point.z = 0.0;

    // Test the transformed point
    if (p.point.x < min_x_ || p.point.x > max_x_ || p.point.y < min_y_ || p.point.y > max_y_)
    {
      continue;
    }

    // Get transform (if any)
    for (int z = 0; z < z_repeats_; ++z)
    {
      geometry_msgs::msg::PointStamped p_out;
      if (do_transform)
      {
        //p.header.stamp = ros::Time(0);
        p.header.frame_id = scan_.header.frame_id;
        p.point.z = z * z_offset_;
        try
        {
          tf2_buffer_->transform(p, p_out, transform_frame_);
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_ERROR(LOGGER, "%s", ex.what());
          rclcpp::sleep_for(std::chrono::seconds(1));
          continue;
        }
      }
      else
      {
        p_out = p;
      }

      // This is a valid point, move it forward
      (cloud_iter + line_point_count)[X] = p_out.point.x;
      (cloud_iter + line_point_count)[Y] = p_out.point.y;
      (cloud_iter + line_point_count)[Z] = p_out.point.z;
      ++line_point_count;
    }
  }

  // Resize clouds
  cloud.height = 1;
  cloud.width  = line_point_count;
}

void ScanFinder::extractObservation(const sensor_msgs::msg::PointCloud2& cloud,
                                    robot_calibration_msgs::msg::CalibrationData * msg)
{
  if (static_cast<int>(cloud.width) == 0)
  {
    RCLCPP_WARN(LOGGER, "No points in observation, skipping");
    return;
  }

  RCLCPP_INFO(LOGGER, "Got %d points for observation", cloud.width);

  // Create PointCloud2 to publish
  sensor_msgs::msg::PointCloud2 viz_cloud;
  viz_cloud.width = 0;
  viz_cloud.height = 0;
  viz_cloud.header.stamp = clock_->now();
  viz_cloud.header.frame_id = cloud.header.frame_id;
  sensor_msgs::PointCloud2Modifier cloud_mod(viz_cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(cloud.width);

  // Setup observation
  int idx_cam = msg->observations.size();
  msg->observations.resize(msg->observations.size() + 1);
  msg->observations[idx_cam].sensor_name = laser_sensor_name_;

  // Fill in observation
  sensor_msgs::PointCloud2Iterator<float> viz_cloud_iter(viz_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud, "x");
  for (size_t i = 0; i < cloud.width; ++i)
  {
    // Get 3d point
    geometry_msgs::msg::PointStamped rgbd;
    rgbd.point.x = (xyz + i)[X];
    rgbd.point.y = (xyz + i)[Y];
    rgbd.point.z = (xyz + i)[Z];

    // Add to observation
    msg->observations[idx_cam].features.push_back(rgbd);

    // Copy to cloud for publishing
    viz_cloud_iter[0] = rgbd.point.x;
    viz_cloud_iter[1] = rgbd.point.y;
    viz_cloud_iter[2] = rgbd.point.z;
    ++viz_cloud_iter;
  }

  // Add debug cloud to message
  if (output_debug_)
  {
    msg->observations[idx_cam].cloud = cloud;
  }

  // Publish debug info
  publisher_->publish(viz_cloud);
}

}  // namespace robot_calibration

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_calibration::ScanFinder, robot_calibration::FeatureFinder)
