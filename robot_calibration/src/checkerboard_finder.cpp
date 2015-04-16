/*
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

#include <robot_calibration/capture/checkerboard_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace robot_calibration
{

CheckerboardFinder::CheckerboardFinder(ros::NodeHandle & n) : 
  FeatureFinder(n),
  waiting_(false)
{
  ros::NodeHandle nh(n, "checkerboard_finder");

  // Setup Scriber
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = n.subscribe(topic_name,
                            1,
                            &CheckerboardFinder::cameraCallback,
                            this);

  // Size of checkerboard
  nh.param<int>("points_x", points_x_, 4);
  nh.param<int>("points_y", points_y_, 5);

  // Should we output debug image/cloud
  nh.param<bool>("debug", output_debug_, false);

  // Publish where checkerboard points were seen
  publisher_ = n.advertise<sensor_msgs::PointCloud2>("checkerboard_points", 10);
}

void CheckerboardFinder::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if (waiting_)
  {
    cloud_ptr_ = cloud;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout
bool CheckerboardFinder::waitForCloud()
{
  waiting_ = true;
  int count = 0;
  while (waiting_ && (++count < 20))
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  return !waiting_;
}

bool CheckerboardFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  // Try up to 50 frames
  for (int i = 0; i < 50; ++i)
  {
    if (findInternal(msg))
      return true;
  }
  return false;
}

bool CheckerboardFinder::findInternal(robot_calibration_msgs::CalibrationData * msg)
{
  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

  // Get cloud
  if(!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  // Get an OpenCV image from the cloud
  cv_bridge::CvImagePtr bridge;
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
  pcl_broke_again::toROSMsg (*cloud_ptr_, *image_msg);
  try
  {
    bridge = cv_bridge::toCvCopy(image_msg, "mono8");  // TODO: was rgb8? does this work?
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return false;
  }

  // Find checkerboard
  std::vector<cv::Point2f> points;
  points.resize(points_x_ * points_y_);
  cv::Size checkerboard_size(points_x_, points_y_);
  int found = cv::findChessboardCorners(bridge->image, checkerboard_size,
                                        points, CV_CALIB_CB_ADAPTIVE_THRESH);

  if (found)
  {
    ROS_INFO("Found the checkboard");

    // Create PointCloud2 to publish
    sensor_msgs::PointCloud2 cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = cloud_ptr_->header.frame_id;
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(points_x_ * points_y_);
    sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

    // Set msg size
    msg->observations.resize(2);
    msg->observations[0].sensor_name = "camera";  // TODO: parameterize
    msg->observations[0].features.resize(points_x_ * points_y_);
    msg->observations[1].sensor_name = "arm";     // TODO: parameterize
    msg->observations[1].features.resize(points_x_ * points_y_);

    // Fill in the headers
    rgbd.header.seq = cloud_ptr_->header.seq;
    rgbd.header.frame_id = cloud_ptr_->header.frame_id;
    rgbd.header.stamp.fromNSec(cloud_ptr_->header.stamp * 1e3);  // from pcl_conversion

    world.header.frame_id = "checkerboard";

    // Fill in message
    for (size_t i = 0; i < points.size(); ++i)
    {
      world.point.x = i / points_x_;
      world.point.y = i % points_x_;

      // Get 3d point
      int index = (int)(points[i].y) * cloud_ptr_->width + (int)(points[i].x);
      rgbd.point.x = cloud_ptr_->points[index].x;
      rgbd.point.y = cloud_ptr_->points[index].y;
      rgbd.point.z = cloud_ptr_->points[index].z;

      // Do not accept NANs
      if (isnan(rgbd.point.x) ||
          isnan(rgbd.point.y) ||
          isnan(rgbd.point.z))
      {
        ROS_ERROR_STREAM("NAN point on " << i);
        return false;
      }

      msg->observations[0].features[i] = rgbd;
      msg->observations[1].features[i] = world;

      // Visualize
      iter_cloud[0] = rgbd.point.x;
      iter_cloud[1] = rgbd.point.y;
      iter_cloud[2] = rgbd.point.z;
      ++iter_cloud;
    }

    // Add debug cloud to message
    if (output_debug_)
    {
      pcl::toROSMsg(*cloud_ptr_, msg->observations[0].cloud);
    }

    // Publish results
    publisher_.publish(cloud);

    // Found all points
    return true;
  }

  return false;
}

}  // namespace robot_calibration
