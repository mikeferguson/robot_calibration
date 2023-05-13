/*
 * Copyright (C) 2022-2023 Michael Ferguson
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

#include <robot_calibration/finders/checkerboard_finder.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("checkerboard_finder");

namespace robot_calibration
{

// We use a number of PC2 iterators, define the indexes here
const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

template <typename T>
CheckerboardFinder<T>::CheckerboardFinder() :
  waiting_(false)
{
}

template <typename T>
bool CheckerboardFinder<T>::init(const std::string& name,
                                 std::shared_ptr<tf2_ros::Buffer> buffer,
                                 rclcpp::Node::SharedPtr node)
{
  if (!FeatureFinder::init(name, buffer, node))
  {
    return false;
  }

  clock_ = node->get_clock();

  // Setup subscriber
  std::string topic_name;
  topic_name = node->declare_parameter<std::string>(name + ".topic", name + "/points");
  subscriber_ = node->create_subscription<T>(
    topic_name,
    rclcpp::QoS(1).best_effort().keep_last(1),
    std::bind(&CheckerboardFinder::cameraCallback, this, std::placeholders::_1));

  // Size of checkerboard
  points_x_ = node->declare_parameter<int>(name + ".points_x", 5);
  points_y_ = node->declare_parameter<int>(name + ".points_y", 4);
  square_size_ = node->declare_parameter<double>(name + ".size", 0.0245);
  if (points_x_ % 2 == 1 && points_y_ % 2 == 1)
  {
    RCLCPP_WARN(LOGGER, "Checkerboard is symmetric - orientation estimate can be wrong");
  }

  // Should we include debug image/cloud in observations
  output_debug_ = node->declare_parameter<bool>(name + ".debug", false);

  // Name of checkerboard frame that will be used during optimization
  frame_id_ = node->declare_parameter<std::string>(name + ".frame_id", "checkerboard");

  // Name of the sensor model that will be used during optimization
  camera_sensor_name_ = node->declare_parameter<std::string>(name + ".camera_sensor_name", "camera");
  chain_sensor_name_ = node->declare_parameter<std::string>(name + ".chain_sensor_name_id", "arm");

  // Publish where checkerboard points were seen
  publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(name + "_points", 10);

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(name, node, LOGGER))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

template <typename T>
void CheckerboardFinder<T>::cameraCallback(typename T::ConstSharedPtr msg)
{
  if (waiting_)
  {
    msg_ = *msg;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout
template <typename T>
bool CheckerboardFinder<T>::waitForMsg()
{
  // Stored as weak pointer, need to grab a real shared pointer
  auto node = node_ptr_.lock();
  if (!node)
  {
    RCLCPP_ERROR(LOGGER, "Unable to get rclcpp::Node lock");
    return false;
  }

  // Initial wait cycle so that camera is definitely up to date.
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
  RCLCPP_ERROR(LOGGER, "Failed to get message");
  return !waiting_;
}

template <typename T>
bool CheckerboardFinder<T>::find(robot_calibration_msgs::msg::CalibrationData * msg)
{
  // Try up to 50 frames
  for (int i = 0; i < 50; ++i)
  {
    // temporary copy of msg, so we throw away all changes if findInternal() returns false
    robot_calibration_msgs::msg::CalibrationData tmp_msg(*msg);
    if (findInternal(&tmp_msg))
    {
      *msg = tmp_msg;
      return true;
    }
  }
  return false;
}

template <>
bool CheckerboardFinder<sensor_msgs::msg::PointCloud2>::findInternal(robot_calibration_msgs::msg::CalibrationData * msg)
{
  geometry_msgs::msg::PointStamped rgbd;
  geometry_msgs::msg::PointStamped world;

  // Get cloud
  if (!waitForMsg())
  {
    RCLCPP_ERROR(LOGGER, "No point cloud data");
    return false;
  }

  if (msg_.height == 1)
  {
    RCLCPP_ERROR(LOGGER, "OpenCV does not support unorganized cloud/image.");
    return false;
  }

  // Get an image message from point cloud
  sensor_msgs::msg::Image::SharedPtr image_msg(new sensor_msgs::msg::Image);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> rgb(msg_, "rgb");
  image_msg->encoding = "bgr8";
  image_msg->height = msg_.height;
  image_msg->width = msg_.width;
  image_msg->step = image_msg->width * sizeof (uint8_t) * 3;
  image_msg->data.resize(image_msg->step * image_msg->height);
  for (size_t y = 0; y < msg_.height; y++)
  {
    for (size_t x = 0; x < msg_.width; x++)
    {
      uint8_t* pixel = &(image_msg->data[y * image_msg->step + x * 3]);
      pixel[0] = rgb[0];
      pixel[1] = rgb[1];
      pixel[2] = rgb[2];
      ++rgb;
    }
  }

  std::vector<cv::Point2f> points;
  if (findCheckerboardPoints(image_msg, points))
  {
    RCLCPP_INFO(LOGGER, "Found the checkboard");

    // Create PointCloud2 to publish
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.header.stamp = clock_->now();
    cloud.header.frame_id = msg_.header.frame_id;
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
    rgbd.header = msg_.header;
    world.header.frame_id = frame_id_;

    // Fill in message
    sensor_msgs::PointCloud2ConstIterator<float> xyz(msg_, "x");
    for (size_t i = 0; i < points.size(); ++i)
    {
      world.point.x = (i % points_x_) * square_size_;
      world.point.y = (i / points_x_) * square_size_;

      // Get 3d point
      int index = (int)(points[i].y) * msg_.width + (int)(points[i].x);
      rgbd.point.x = (xyz + index)[X];
      rgbd.point.y = (xyz + index)[Y];
      rgbd.point.z = (xyz + index)[Z];

      // Do not accept NANs
      if (std::isnan(rgbd.point.x) ||
          std::isnan(rgbd.point.y) ||
          std::isnan(rgbd.point.z))
      {
        RCLCPP_ERROR_STREAM(LOGGER, "NAN point on " << i);
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
      msg->observations[idx_cam].cloud = msg_;
    }

    // Publish results
    publisher_->publish(cloud);

    // Found all points
    return true;
  }

  return false;
}

template <typename T>
bool CheckerboardFinder<T>::findCheckerboardPoints(sensor_msgs::msg::Image::SharedPtr image,
                                                   std::vector<cv::Point2f>& points)
{
  // Get an OpenCV image from the cloud
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(image, "mono8");  // TODO: was rgb8? does this work?
  }
  catch(cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(LOGGER, "Conversion failed");
    return false;
  }

  // Find checkerboard
  points.resize(points_x_ * points_y_);
  cv::Size checkerboard_size(points_x_, points_y_);
  return cv::findChessboardCorners(bridge->image, checkerboard_size,
                                   points, cv::CALIB_CB_ADAPTIVE_THRESH);
}

}  // namespace robot_calibration

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_calibration::CheckerboardFinder<sensor_msgs::msg::PointCloud2>, robot_calibration::FeatureFinder)
