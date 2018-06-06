/*
 * Copyright (C) 2018 Michael Ferguson
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

// Author: Niharika Arora, Michael Ferguson

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <robot_calibration/capture/plane_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::PlaneFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

PlaneFinder::PlaneFinder() :
  tfListener_(tfBuffer_), waiting_(false)
{
}

bool PlaneFinder::init(const std::string& name,
                       ros::NodeHandle & nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  // We subscribe to a PointCloud2
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &PlaneFinder::cameraCallback,
                             this);

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");

  // Maximum number of valid points to include in the observation
  nh.param<double>("points_max", points_max_, 60);

  // Frame to transform point cloud into before applying limits below
  //   if specified as "none", cloud will be processed in sensor frame
  nh.param<std::string>("transform_frame", transform_frame_, "base_link");

  // Valid points must lie within this box, in the transform_frame
  nh.param<double>("min_x", min_x_, -2.0);
  nh.param<double>("max_x", max_x_, 2.0);
  nh.param<double>("min_y", min_y_, -2.0);
  nh.param<double>("max_y", max_y_, 2.0);
  nh.param<double>("min_z", min_z_, 0.0);
  nh.param<double>("max_z", max_z_, 2.0);

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Publish the observation as a PointCloud2
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  // Make sure we have CameraInfo before starting
  if (!depth_camera_manager_.init(nh))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

void PlaneFinder::cameraCallback(const sensor_msgs::PointCloud2& cloud)
{
  if (waiting_)
  {
    cloud_ = cloud;
    waiting_ = false;
  }
}

bool PlaneFinder::waitForCloud()
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
  ROS_ERROR("Failed to get cloud");
  return !waiting_;
}

bool PlaneFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  if (!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  //  Remove any point that is invalid or not with our tolerance
  size_t num_points = cloud_.width * cloud_.height;
  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud_, "x");
  sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud_, "x");

  bool do_transform = transform_frame_ != "none";  // This can go away once the cloud gets transformed outside loop 
  size_t j = 0;
  for (size_t i = 0; i < num_points; i++)
  {
    geometry_msgs::PointStamped p;
    p.point.x = (xyz + i)[X];
    p.point.y = (xyz + i)[Y];
    p.point.z = (xyz + i)[Z];

    // Remove the NaNs in the point cloud
    if (!std::isfinite(p.point.x) || !std::isfinite(p.point.y) || !std::isfinite(p.point.z))
      continue;

    // Remove the points immediately in front of the camera in the point cloud
    // NOTE : This is to handle sensors that publish zeros instead of NaNs in the point cloud
    if (p.point.z == 0)
      continue;

    // Get transform (if any)
    geometry_msgs::PointStamped p_out;
    if (do_transform)
    {
      p.header.stamp = ros::Time(0);
      p.header.frame_id = cloud_.header.frame_id;
      try
      {
        tfBuffer_.transform(p, p_out, transform_frame_);
      }
      catch (tf2::TransformException ex)
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

    // Test the transformed point
    if (p_out.point.x < min_x_ || p_out.point.x > max_x_ || p_out.point.y < min_y_ || p_out.point.y > max_y_ ||
        p_out.point.z < min_z_ || p_out.point.z > max_z_)
    {
      continue;
    }

    // This is a valid point, move it forward
    (cloud_iter + j)[X] = (xyz + i)[X];
    (cloud_iter + j)[Y] = (xyz + i)[Y];
    (cloud_iter + j)[Z] = (xyz + i)[Z];
    j++;
  }
  cloud_.height = 1;
  cloud_.width  = j;
  cloud_.data.resize(cloud_.width * cloud_.point_step);

  // Determine number of points to output
  size_t points_total = std::min(static_cast<size_t>(points_max_), j);
  ROS_INFO_STREAM("Got " << j << " points from plane, using " << points_total);

  // Create PointCloud2 to publish
  sensor_msgs::PointCloud2 viz_cloud;
  viz_cloud.width = 0;
  viz_cloud.height = 0;
  viz_cloud.header.stamp = ros::Time::now();
  viz_cloud.header.frame_id = cloud_.header.frame_id;
  sensor_msgs::PointCloud2Modifier cloud_mod(viz_cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(points_total);
  sensor_msgs::PointCloud2Iterator<float> viz_cloud_iter(viz_cloud, "x");

  // Setup observation
  int idx_cam = msg->observations.size();
  msg->observations.resize(msg->observations.size() + 1);
  msg->observations[idx_cam].sensor_name = camera_sensor_name_;
  msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();

  // Fill in observation
  size_t step = cloud_.width / points_total;
  size_t index = step / 2;
  for (size_t i = 0; index < cloud_.width; i++)
  {
    // Get (untransformed) 3d point
    geometry_msgs::PointStamped rgbd;
    rgbd.point.x = (xyz + index)[X];
    rgbd.point.y = (xyz + index)[Y];
    rgbd.point.z = (xyz + index)[Z];

    // Add to observation
    msg->observations[idx_cam].features.push_back(rgbd);

    // Copy to cloud for publishing
    viz_cloud_iter[0] = rgbd.point.x;
    viz_cloud_iter[1] = rgbd.point.y;
    viz_cloud_iter[2] = rgbd.point.z;
    ++viz_cloud_iter;

    // Next point
    index += step;
  }

  // Add debug cloud to message
  if (output_debug_)
  {
    msg->observations[idx_cam].cloud = cloud_;
  }

  // Publish debug info
  publisher_.publish(viz_cloud);

  // Report success
  return true;
}

}  // namespace robot_calibration
