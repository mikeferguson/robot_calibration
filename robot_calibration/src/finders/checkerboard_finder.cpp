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

#include <pluginlib/class_list_macros.h>
#include <robot_calibration/capture/checkerboard_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::CheckerboardFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

// We use a number of PC2 iterators, define the indexes here
const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

CheckerboardFinder::CheckerboardFinder() :
  waiting_(false)
{
}

bool CheckerboardFinder::init(const std::string& name,
                              ros::NodeHandle & nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  // Setup Scriber
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &CheckerboardFinder::cameraCallback,
                             this);

  image_transport::ImageTransport it(nh);
  pub_checkerboard_ = it.advertise("detected_points", 1);     
  ROS_INFO_STREAM("Publishing detected corners on topic: " << pub_checkerboard_.getTopic());                      

  // Size of checkerboard
  nh.param<int>("points_x", points_x_, 5);
  nh.param<int>("points_y", points_y_, 4);
  nh.param<double>("size", square_size_, 0.0245);

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Name of checkerboard frame that will be used during optimization
  nh.param<std::string>("frame_id", frame_id_, "checkerboard");

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

  // Publish where checkerboard points were seen
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  ROS_INFO_STREAM("points_x: " << points_x_ << "\npoints_y: " << points_y_ << "\nsize: " << square_size_ << "\nframe_id: " << frame_id_<< "\ncamera_sensor_name_: " << camera_sensor_name_ << "\nchain_sensor_name: " << chain_sensor_name_);

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(nh))
  {
    // Error will have been printed by manager
    return false;
  } 

  return true;
}

void CheckerboardFinder::cameraCallback(const sensor_msgs::PointCloud2& cloud)
{
  if (waiting_)
  {
    cloud_ = cloud;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout
bool CheckerboardFinder::waitForCloud()
{
  // Initial wait cycle so that camera is definitely up to date.
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

bool CheckerboardFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  const int count = 50;
  ROS_INFO_STREAM("Trying to detect checkboard for max " << count << " times.");
  // Try up to 50 frames
  for (int i = 0; i < count; ++i)
  {
    // temporary copy of msg, so we throw away all changes if findInternal() returns false
    robot_calibration_msgs::CalibrationData tmp_msg(*msg);
    if (findInternal(&tmp_msg))
    {
      *msg = tmp_msg;
      return true;
    } else {
      ROS_INFO_STREAM("No checkboard found in iteration: " << i);
    }
  }
  return false;
}

bool CheckerboardFinder::findInternal(robot_calibration_msgs::CalibrationData * msg)
{
  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

  // Get cloud
  if (!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  // Get an image message from point cloud
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> rgb(cloud_, "rgb");
  image_msg->encoding = "bgr8";
  image_msg->height = cloud_.height;
  image_msg->width = cloud_.width;
  image_msg->step = image_msg->width * sizeof (uint8_t) * 3;
  image_msg->data.resize(image_msg->step * image_msg->height);
  for (size_t y = 0; y < cloud_.height; y++)
  {
    for (size_t x = 0; x < cloud_.width; x++)
    {
      uint8_t* pixel = &(image_msg->data[y * image_msg->step + x * 3]);
      pixel[0] = rgb[0];
      pixel[1] = rgb[1];
      pixel[2] = rgb[2];
      ++rgb;
    }
  }

  // Get an OpenCV image from the cloud
  cv_bridge::CvImagePtr bridge;
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

  // draw detected corners
  cv::Mat corners_drawn(bridge->image.size(), CV_8UC3);
  cv::cvtColor(bridge->image, corners_drawn, cv::COLOR_GRAY2BGR);
  cv::drawChessboardCorners(corners_drawn, checkerboard_size, cv::Mat(points), found);

  pub_checkerboard_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", corners_drawn).toImageMsg());

  if (found)
  {
    ROS_INFO("Found the checkboard");

    // Create PointCloud2 to publish
    sensor_msgs::PointCloud2 cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = cloud_.header.frame_id;
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(points_x_ * points_y_);
    sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

    // Set msg size
    int idx_cam = msg->observations.size() + 0;
    int idx_chain = msg->observations.size() + 1;
    msg->observations.resize(msg->observations.size() + 2);
    msg->observations[idx_cam].sensor_name = camera_sensor_name_;
    msg->observations[idx_chain].sensor_name = chain_sensor_name_;
         
    msg->observations[idx_cam].features.resize(points_x_ * points_y_);
    msg->observations[idx_chain].features.resize(points_x_ * points_y_);


    // Fill in the headers
    rgbd.header = cloud_.header;
    world.header.frame_id = frame_id_;

    // Fill in message
    sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud_, "x");
    for (size_t i = 0; i < points.size(); ++i)
    {
      world.point.x = (i % points_x_) * square_size_;
      world.point.y = (i / points_x_) * square_size_;

      // Get 3d point
      int index = (int)(points[i].y) * cloud_.width + (int)(points[i].x);
      rgbd.point.x = (xyz + index)[X];
      rgbd.point.y = (xyz + index)[Y];
      rgbd.point.z = (xyz + index)[Z];

      // Do not accept NANs
      if (std::isnan(rgbd.point.x) ||
          std::isnan(rgbd.point.y) ||
          std::isnan(rgbd.point.z))
      {
        ROS_ERROR_STREAM("NAN point on " << i);
        return false;
      }

      // Do not accept (0.0, 0.0, 0.0)
      const double eps = static_cast<double>(10.0) * std::numeric_limits<double>::epsilon();
      if ((std::fabs(rgbd.point.x) < eps) || (std::fabs(rgbd.point.y) < eps) || (std::fabs(rgbd.point.z) < eps))
      {
        ROS_ERROR_STREAM("(0.0, 0.0, 0.0) point on " << i);
        return false;
      }

      msg->observations[idx_cam].features[i] = rgbd;
      msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
      msg->observations[idx_chain].features[i] = world;

      // Visualize
      iter_cloud[0] = rgbd.point.x;
      iter_cloud[1] = rgbd.point.y;
      iter_cloud[2] = rgbd.point.z;
      ++iter_cloud;
    }

    // Add debug cloud to message
    if (output_debug_)
    {
      msg->observations[idx_cam].cloud = cloud_;
    }

    // Publish results
    publisher_.publish(cloud);

    // Found all points
    return true;
  }

  return false;
}

}  // namespace robot_calibration
