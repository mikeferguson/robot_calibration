/*
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

#include <ubr_calibration/capture/checkerboard_finder.h>

namespace ubr_calibration
{

CheckerboardFinder::CheckerboardFinder(ros::NodeHandle & n) : 
  FeatureFinder(n), waiting_(false)
{
  subscriber_ = n.subscribe("/head_camera/depth_registered/points",
                            1,
                            &CheckerboardFinder::cameraCallback,
                            this);

  n.param<int>("checkerboard_finder_points_x", points_x_, 4);
  n.param<int>("checkerboard_finder_points_y", points_y_, 5);
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

bool CheckerboardFinder::find(ubr_calibration::CalibrationData * msg)
{
  // Try up to 50 frames
  for (int i = 0; i < 50; ++i)
  {
    if (findInternal(msg))
      return true;
  }
  return false;
}

bool CheckerboardFinder::findInternal(ubr_calibration::CalibrationData * msg)
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

    // Set msg size
    msg->rgbd_observations.resize(points_x_ * points_y_);
    msg->world_observations.resize(points_x_ * points_y_);

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

      msg->rgbd_observations[i] = rgbd;
      msg->world_observations[i] = world;
    }
    pcl::toROSMsg(*cloud_ptr_, msg->cloud);

    // Found all points
    return true;
  }

  return false;
}

}  // namespace ubr_calibration
