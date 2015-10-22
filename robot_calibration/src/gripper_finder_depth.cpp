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

#include <robot_calibration/capture/gripper_finder_depth.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/rgbd/rgbd.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

GripperFinderDepth::GripperFinderDepth(ros::NodeHandle & nh) :
  FeatureFinder(nh),
  waiting_(false)
{
//std::string topic_name;
//nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe("/head_camera/depth_downsample/image_raw",
                            1,
                            &GripperFinderDepth::cameraCallback,
                            this);

 //nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
 // nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "ground");

  publisher_ = nh.advertise<sensor_msgs::Image>("gripper_depth_points", 10);
  if (!depth_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }
}

void GripperFinderDepth::cameraCallback(const sensor_msgs::Image& cloud)
{
  if (waiting_)
  {
    cloud_ = cloud;
    waiting_ = false;
  }
}

bool GripperFinderDepth::waitForCloud()
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


bool GripperFinderDepth::find(robot_calibration_msgs::CalibrationData * msg)
{
  //geometry_msgs::PointStamped rgbd;
  //geometry_msgs::PointStamped world;

  //if (!waitForCloud())
//  {
  //  ROS_ERROR("No point cloud data");
  //  return false;
//  }
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(cloud_, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  cv::Mat img_eroded;
  cv::erode(cv_ptr->image, cv_ptr->image, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  
  cv::imshow("Image window", cv_ptr->image);
   cv::waitKey(1000);
  /*
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
  msg->observations.resize(2);
  msg->observations[0].sensor_name = camera_sensor_name_;
  msg->observations[0].features.resize(points_total);
  msg->observations[1].sensor_name = chain_sensor_name_;
  msg->observations[1].features.resize(points_total);

  size_t step = cloud_.width/points_total;
  size_t k = 0;

  for (size_t i = step; i < cloud_.width; i +=step)
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
    msg->observations[0].features[i] = rgbd;
    msg->observations[0].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
    msg->observations[1].features[i] = world;

    iter_cloud[0] = rgbd.point.x;
    iter_cloud[1] = rgbd.point.y;
    iter_cloud[2] = rgbd.point.z;
    ++iter_cloud;
  } */
  
  publisher_.publish(cv_ptr->toImageMsg());
  return true;
}
}  // namespace robot_calibration
