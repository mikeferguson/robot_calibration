/*
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

// Author: Niharika Arora

#include <robot_calibration/capture/plane_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

PlaneFinder::PlaneFinder(ros::NodeHandle & nh) :
    FeatureFinder(nh),
    waiting_(false)
{
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &PlaneFinder::cameraCallback,
                             this);

  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<double>("points_max", points_max_, 60);
  nh.param<double>("min_y", min_y_, 0);
  nh.param<double>("max_y", max_y_, 0);
  nh.param<double>("min_z", min_z_, 0);
  nh.param<double>("max_z", max_z_, 0);
  nh.param<std::string>("transform_frame", transform_frame_, "base_link");

  publisher_ = nh.advertise<sensor_msgs::PointCloud2>("plane_points", 10);
  if (!depth_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }

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
  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

  if (!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  //  Remove NaNs in the point cloud
  size_t num_points = cloud_.width * cloud_.height;
  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter(cloud_, "x");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  try
  {

    transformStamped = tfBuffer.lookupTransform("base_link", cloud_.header.frame_id,
                             ros::Time(0), ros::Duration(10.0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  size_t j = 0;
  for (size_t i = 0; i < num_points; i++)
  {
    geometry_msgs::PointStamped p;
    p.point.x = (xyz + i)[X];
    p.point.y = (xyz + i)[Y];
    p.point.z = (xyz + i)[Z];
    p.header.stamp = ros::Time(0);
    p.header.frame_id = cloud_.header.frame_id;

    geometry_msgs::PointStamped p_out;
    try
    {
      tfBuffer.transform(p, p_out, transform_frame_);
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // Remove the NaNs in the point cloud
    if (!std::isfinite(p.point.x) || !std::isfinite(p.point.y) || !std::isfinite(p.point.z))
      continue;

    // Remove the points immediately in front of te camera in the point cloud
    // NOTE : This is to handle sensors that publish zeros instead of NaNs in the point cloud
    if (p.point.z == 0)
      continue;

    // Transformed point in the transformed frame (default is base_link))
    if (p_out.point.z < min_z_ or p_out.point.z > max_z_ or p_out.point.y < min_y or p_out.point.y > max_y)
    {
      continue;
    }

    (iter + j)[X] = (xyz + i)[X];
    (iter + j)[Y] = (xyz + i)[Y];
    (iter + j)[Z] = (xyz + i)[Z];
    j++;
  }

  cloud_.height = 1;
  cloud_.width  = j;
  cloud_.data.resize(cloud_.width * cloud_.point_step);

  size_t points_total = std::min(static_cast<size_t>(points_max_), j);
  std::vector<cv::Point2f> points;
  points.resize(points_total);

  // Create PointCloud2 to publish
  sensor_msgs::PointCloud2 cloud;
  cloud.width = 0;
  cloud.height = 0;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = cloud_.header.frame_id;
  sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(points_total);
  sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

  // Set msg size
  int idx_cam = msg->observations.size() + 0;
  msg->observations.resize(msg->observations.size() + 1);
  msg->observations[idx_cam].sensor_name = camera_sensor_name_;
  msg->observations[idx_cam].features.resize(points_total);

  size_t step = cloud_.width/(points_total);
  size_t k = 0;

  for (size_t i = step; i < cloud_.width && k < points_total; i += step)
  {
    points[k].x = i;
    k++;
  }

  for (size_t i = 0; i < points.size(); i++)
  {
    // Get 3d point
    int index = static_cast<int>(points[i].x);
    rgbd.point.x = (xyz + index)[X];
    rgbd.point.y = (xyz + index)[Y];
    rgbd.point.z = (xyz + index)[Z];
    msg->observations[idx_cam].features[i] = rgbd;
    msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();

    iter_cloud[0] = rgbd.point.x;
    iter_cloud[1] = rgbd.point.y;
    iter_cloud[2] = rgbd.point.z;
    ++iter_cloud;
  }

  // Publish debug info
  msg->observations[idx_cam].cloud = cloud_;
  publisher_.publish(cloud);
  return true;
}
}  // namespace robot_calibration
