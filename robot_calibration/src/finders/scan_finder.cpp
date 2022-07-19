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
#include <pluginlib/class_list_macros.hpp>
#include <robot_calibration/capture/scan_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::ScanFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

ScanFinder::ScanFinder() :
  tf_listener_(tf_buffer_), waiting_(false)
{
}

bool ScanFinder::init(const std::string& name,
                       ros::NodeHandle & nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  // We subscribe to a LaserScan
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/scan");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &ScanFinder::scanCallback,
                             this);

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("sensor_name", laser_sensor_name_, "laser");

  // Frame to transform point cloud into before applying limits below
  //   if specified as "none", cloud will be processed in sensor frame
  nh.param<std::string>("transform_frame", transform_frame_, "base_link");

  // It is assumed that the laser scanner operates in the X,Y plane
  // Valid points must lie within this box, in the laser frame
  nh.param<double>("min_x", min_x_, -2.0);
  nh.param<double>("max_x", max_x_, 2.0);
  nh.param<double>("min_y", min_y_, -2.0);
  nh.param<double>("max_y", max_y_, 2.0);

  // Repeat points in the Z plane a number of times at fixed distance
  nh.param<int>("z_repeats", z_repeats_, 10);
  nh.param<double>("z_offset", z_offset_, 0.1);

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Publish the observation as a PointCloud2
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  return true;
}

void ScanFinder::scanCallback(const sensor_msgs::LaserScan& scan)
{
  if (waiting_)
  {
    scan_ = scan;
    waiting_ = false;
  }
}

bool ScanFinder::waitForScan()
{
  ros::Duration(1/10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  ROS_ERROR("Failed to get scan");
  return !waiting_;
}

bool ScanFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  if (!waitForScan())
  {
    ROS_ERROR("No laser scan data");
    return false;
  }

  // Extract the points corresponding to the line
  sensor_msgs::PointCloud2 cloud;
  extractPoints(cloud);
  extractObservation(cloud, msg);

  // Report success
  return true;
}

void ScanFinder::extractPoints(sensor_msgs::PointCloud2& cloud)
{
  bool do_transform = transform_frame_ != "none";

  // Reset cloud
  cloud.width = 0;
  cloud.height = 0;
  cloud.header.stamp = ros::Time::now();
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
    geometry_msgs::PointStamped p;
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
      geometry_msgs::PointStamped p_out;
      if (do_transform)
      {
        p.header.stamp = ros::Time(0);
        p.header.frame_id = scan_.header.frame_id;
        p.point.z = z * z_offset_;
        try
        {
          tf_buffer_.transform(p, p_out, transform_frame_);
        }
        catch (tf2::TransformException& ex)
        {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
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

void ScanFinder::extractObservation(const sensor_msgs::PointCloud2& cloud,
                                    robot_calibration_msgs::CalibrationData * msg)
{
  if (static_cast<int>(cloud.width) == 0)
  {
    ROS_WARN("No points in observation, skipping");
    return;
  }

  ROS_INFO_STREAM("Got " << cloud.width << " points for observation");

  // Create PointCloud2 to publish
  sensor_msgs::PointCloud2 viz_cloud;
  viz_cloud.width = 0;
  viz_cloud.height = 0;
  viz_cloud.header.stamp = ros::Time::now();
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
    geometry_msgs::PointStamped rgbd;
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
  publisher_.publish(viz_cloud);
}

}  // namespace robot_calibration
