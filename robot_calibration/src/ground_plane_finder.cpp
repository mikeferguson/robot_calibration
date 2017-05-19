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

#include <robot_calibration/capture/ground_plane_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <math.h>

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

GroundPlaneFinder::GroundPlaneFinder(ros::NodeHandle & nh) :
  FeatureFinder(nh),
  waiting_(false)
{
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &GroundPlaneFinder::cameraCallback,
                             this);

  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "cameraground");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "ground");
  nh.param<double>("points_max", points_max_, 60);
  nh.param<double>("max_z", max_z_, 0);

  publisher_ = nh.advertise<sensor_msgs::PointCloud2>("ground_plane_points", 10);
  if (!depth_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }
}

void GroundPlaneFinder::cameraCallback(const sensor_msgs::PointCloud2& cloud)
{
  if (waiting_)
  {
    cloud_ = cloud;
    waiting_ = false;
  }
}

bool GroundPlaneFinder::waitForCloud()
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


bool GroundPlaneFinder::find(robot_calibration_msgs::CalibrationData * msg)
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

  size_t j = 0;
  for (size_t i = 0; i < num_points; i++)
  {
    geometry_msgs::Point p;
    p.x = (xyz + i)[X];
    p.y = (xyz + i)[Y];
    p.z = (xyz + i)[Z];

    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;

    if (p.z >max_z_ && max_z_ != 0)
      continue;
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
  sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud_, "x");

  // Set msg size
  int idx_cam = msg->observations.size() + 0;
  int idx_chain = msg->observations.size() + 1;
  msg->observations.resize(msg->observations.size() + 2);
  msg->observations[idx_cam].sensor_name = camera_sensor_name_;
  msg->observations[idx_chain].sensor_name = chain_sensor_name_;

  msg->observations[idx_cam].features.resize(points_total);
  msg->observations[idx_chain].features.resize(points_total);

  size_t step = cloud_.width/(points_total);
  size_t k = 0;

  for (size_t i = step; i < cloud_.width && k < points_total; i += step)
  {
    points[k].x = i;
    k++;
  }

  for (size_t i = 0; i < points.size(); i++)
  {
    // for ground plane world can just be zero as we are concerned only with z
    world.point.x = 0;
    world.point.y = 0;
    world.point.z = 0;

    // Get 3d point
    int index = static_cast<int>(points[i].x);
    rgbd.point.x = (xyz + index)[X];
    rgbd.point.y = (xyz + index)[Y];
    rgbd.point.z = (xyz + index)[Z];
    msg->observations[idx_cam].features[i] = rgbd;
    msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
    msg->observations[idx_chain].features[i] = world;

    iter_cloud[0] = rgbd.point.x;
    iter_cloud[1] = rgbd.point.y;
    iter_cloud[2] = rgbd.point.z;
    ++iter_cloud;
  }

  publisher_.publish(cloud);
  return true;
}
}  // namespace robot_calibration
