/*
 * Copyright (C) 2022 Michael Ferguson
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

#include <math.h>
#include <robot_calibration/capture/led_finder.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("led_finder");

namespace robot_calibration
{

// We use a number of PC2 iterators, define the indexes here
const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;
const unsigned R = 0;
const unsigned G = 1;
const unsigned B = 2;

double distancePoints(
  const geometry_msgs::msg::Point p1,
  const geometry_msgs::msg::Point p2)
{
  return std::sqrt((p1.x-p2.x) * (p1.x-p2.x) +
                   (p1.y-p2.y) * (p1.y-p2.y) +
                   (p1.z-p2.z) * (p1.z-p2.z));
}

LedFinder::LedFinder() :
  waiting_(false)
{
}

bool LedFinder::init(const std::string& name,
                     std::shared_ptr<tf2_ros::Buffer> buffer,
                     rclcpp::Node::SharedPtr node)
{
  if (!FeatureFinder::init(name, buffer, node))
  {
    return false;
  }

  clock_ = node->get_clock();

  // Setup the action client
  std::string topic_name =
    node->declare_parameter<std::string>(name + ".action", "/gripper_led_action");
  client_ = rclcpp_action::create_client<LedAction>(node, topic_name);
  RCLCPP_INFO(LOGGER, "Waiting for %s...", topic_name.c_str());
  client_->wait_for_action_server();

  // Setup subscriber
  topic_name = node->declare_parameter<std::string>(name + ".topic", name + "/points");
  subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name, 1, std::bind(&LedFinder::cameraCallback, this, std::placeholders::_1));

  // Publish where LEDs were seen
  publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(name + "_points", 10);

  // Maximum distance LED can be from expected pose
  max_error_ = node->declare_parameter<double>(name + ".max_error", 0.1);
  // Maximum relative difference between two LEDs
  max_inconsistency_ = node->declare_parameter<double>(name + ".max_inconsistency", 0.01);

  // Parameters for detection
  threshold_ = node->declare_parameter<double>(name + ".threshold", 1000.0);
  max_iterations_ = node->declare_parameter<int>(name + ".max_iterations", 50);

  // Should we output debug image/cloud
  output_debug_ = node->declare_parameter<bool>(name + ".debug", false);

  // Name of the sensor model that will be used during optimization
  camera_sensor_name_ = node->declare_parameter<std::string>(name + ".camera_sensor_name", "camera");
  chain_sensor_name_ = node->declare_parameter<std::string>(name + ".chain_sensor_name", "arm");

  // Parameters for LEDs themselves
  std::string gripper_led_frame =
    node->declare_parameter<std::string>(name + ".gripper_led_frame", "wrist_roll_link");
  std::vector<std::string> led_names =
    node->declare_parameter<std::vector<std::string>>(name + ".leds", std::vector<std::string>());
  for (auto led_name : led_names)
  {
    int code = node->declare_parameter<int>(name + "." + led_name + ".code", 0);
    codes_.push_back(code);
    codes_.push_back(0);  // assumes "0" is code for "OFF"

    double x, y, z;
    x = node->declare_parameter<double>(name + "." + led_name + ".x", 0.0);
    y = node->declare_parameter<double>(name + "." + led_name + ".y", 0.0);
    z = node->declare_parameter<double>(name + "." + led_name + ".z", 0.0);
    trackers_.push_back(CloudDifferenceTracker(gripper_led_frame, x, y, z));

    // Publisher
    topic_name = node->declare_parameter<std::string>(name + "." + led_name + ".topic", led_name + "_debug");
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub =
      node->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    tracker_publishers_.push_back(pub);
  }

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(name, node, LOGGER))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

void LedFinder::cameraCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (waiting_)
  {
    cloud_ = *cloud;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout.
bool LedFinder::waitForCloud()
{
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
    // TODO ros::spinOnce();
  }
  RCLCPP_ERROR(LOGGER, "Failed to get cloud");
  return !waiting_;
}

bool LedFinder::find(robot_calibration_msgs::msg::CalibrationData * msg)
{
  uint8_t code_idx = -1;

  std::vector<geometry_msgs::msg::PointStamped> rgbd;
  std::vector<geometry_msgs::msg::PointStamped> world;

  sensor_msgs::msg::PointCloud2 prev_cloud;

  auto command = LedAction::Goal();
  command.led_code = 0;
  client_->async_send_goal(command);
  // TODO client_->waitForResult(ros::Duration(10.0));

  // Get initial cloud
  if (!waitForCloud())
  {
    return false;
  }
  prev_cloud = cloud_;

  // Initialize difference trackers
  for (size_t i = 0; i < trackers_.size(); ++i)
  {
    trackers_[i].reset(cloud_.height, cloud_.width);
  }

  int cycles = 0;
  while (true)
  {
    // Toggle LED to next state
    code_idx = (code_idx + 1) % codes_.size();
    command.led_code = codes_[code_idx];
    client_->async_send_goal(command);
    // TODO client_->waitForResult(ros::Duration(10.0));

    // Get a point cloud
    if (!waitForCloud())
    {
      return false;
    }

    // Commands are organized as On-Off for each led.
    int tracker = code_idx/2;
    // Even indexes are turning on, Odd are turning off
    double weight = (code_idx%2 == 0) ? 1: -1;

    // Has each point converged?
    bool done = true;
    for (size_t t = 0; t < trackers_.size(); ++t)
    {
      done &= trackers_[t].isFound(cloud_, threshold_);
    }
    // We want to break only if the LED is off, so that pixel is not washed out
    if (done && (weight == -1))
    {
      break;
    }

    // Get expected pose of LED in the cloud frame
    geometry_msgs::msg::PointStamped led;
    led.point = trackers_[tracker].point_;
    led.header.frame_id = trackers_[tracker].frame_;
    try
    {
      tf2_buffer_->transform(led, led, cloud_.header.frame_id);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(LOGGER, "Failed to transform feature to %s", cloud_.header.frame_id.c_str());
      return false;
    }

    // Update the tracker
    trackers_[tracker].process(cloud_, prev_cloud, led.point, max_error_, weight);

    if (++cycles > max_iterations_)
    {
      RCLCPP_ERROR(LOGGER, "Failed to find features before using maximum iterations.");
      return false;
    }

    prev_cloud = cloud_;

    // Publish state of each tracker
    for (size_t i = 0; i < trackers_.size(); i++)
    {
      sensor_msgs::msg::Image image = trackers_[i].getImage();
      tracker_publishers_[i]->publish(image);
    }
  }

  // Create PointCloud2 to publish
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 0;
  cloud.height = 0;
  cloud.header.stamp = clock_->now();
  cloud.header.frame_id = cloud_.header.frame_id;
  sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(4);
  sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

  // Collect Results
  const int CAMERA = 0;
  const int CHAIN = 1;
  robot_calibration_msgs::msg::Observation observations[2];
  observations[CAMERA].sensor_name = camera_sensor_name_;
  observations[CHAIN].sensor_name = chain_sensor_name_;

  for (size_t t = 0; t < trackers_.size(); ++t)
  {
    geometry_msgs::msg::PointStamped rgbd_pt;
    geometry_msgs::msg::PointStamped world_pt;

    // Get point
    if (!trackers_[t].getRefinedCentroid(cloud_, rgbd_pt))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "No centroid for feature " << t);
      return false;
    }

    // Check that point is close enough to expected pose
    try
    {
      tf2_buffer_->transform(rgbd_pt, world_pt, trackers_[t].frame_);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to transform feature to " << trackers_[t].frame_);
      return false;
    }
    double distance = distancePoints(world_pt.point, trackers_[t].point_);
    if (distance > max_error_)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Feature was too far away from expected pose in " << trackers_[t].frame_ << ": " << distance);
      return false;
    }

    // Check that points are consistent with one another
    for (size_t t2 = 0; t2 < t; ++t2)
    {
      double expected = distancePoints(trackers_[t2].point_, trackers_[t].point_);
      double actual = distancePoints(observations[CAMERA].features[t2].point, rgbd_pt.point);
      if (fabs(expected-actual) > max_inconsistency_)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Features not internally consistent: " << expected << " " << actual);
        return false;
      }
    }

    // Push back observation
    observations[CAMERA].features.push_back(rgbd_pt);
    observations[CAMERA].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();

    // Visualize
    iter_cloud[0] = rgbd_pt.point.x;
    iter_cloud[1] = rgbd_pt.point.y;
    iter_cloud[2] = rgbd_pt.point.z;
    ++iter_cloud;

    // Push back expected location of point on robot
    world_pt.header.frame_id = trackers_[t].frame_;
    world_pt.point = trackers_[t].point_;
    observations[CHAIN].features.push_back(world_pt);
  }

  // Final check that all points are valid
  if (observations[CAMERA].features.size() != trackers_.size())
  {
    return false;
  }

  // Add debug cloud to message
  if (output_debug_)
  {
    observations[CAMERA].cloud = cloud_;
  }

  // Copy results to message
  msg->observations.push_back(observations[CAMERA]);
  msg->observations.push_back(observations[CHAIN]);

  // Publish results
  publisher_->publish(cloud);

  return true;
}

LedFinder::CloudDifferenceTracker::CloudDifferenceTracker(
  std::string frame, double x, double y, double z) :
    frame_(frame)
{
  point_.x = x;
  point_.y = y;
  point_.z = z;
}

void LedFinder::CloudDifferenceTracker::reset(size_t height, size_t width)
{
  // Save for creating images
  height_ = height;
  width_ = width;

  // Number of clouds processed.
  count_ = 0;
  // Maximum difference observed
  max_ = -1000.0;
  // Pixel this was observed in
  max_idx_ = -1;

  // Setup difference tracker
  diff_.resize(height * width);
  for (std::vector<double>::iterator it = diff_.begin(); it != diff_.end(); ++it)
  {
    *it = 0.0;
  }
}

// Weight should be +/- 1 typically
bool LedFinder::CloudDifferenceTracker::process(
  sensor_msgs::msg::PointCloud2& cloud,
  sensor_msgs::msg::PointCloud2& prev,
  geometry_msgs::msg::Point& led_point,
  double max_distance,
  double weight)
{
  if ((cloud.width * cloud.height) != diff_.size())
  {
    RCLCPP_ERROR(LOGGER, "Cloud size has changed");
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> rgb(cloud, "rgb");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> prev_rgb(prev, "rgb");

  // We want to compare each point to the expected LED pose,
  // but when the LED is on, the points will be NAN,
  // fall back on most recent distance for these points
  double last_distance = 1000.0;

  // Update each point in the tracker
  const size_t num_points = cloud.data.size() / cloud.point_step;
  int valid = 0;
  int used = 0;
  for (size_t i = 0; i < num_points; i++)
  {
    // If within range of LED pose...
    geometry_msgs::msg::Point p;
    p.x = (xyz + i)[X];
    p.y = (xyz + i)[Y];
    p.z = (xyz + i)[Z];
    double distance = distancePoints(p, led_point);

    if (std::isfinite(distance))
    {
      last_distance = distance;
      valid++;
    }
    else
    {
      distance = last_distance;
    }

    if (!std::isfinite(distance) || distance > max_distance)
    {
      continue;
    }

    // ...and has proper change in sign
    double r = (double)((rgb + i)[R]) - (double)((prev_rgb + i)[R]);
    double g = (double)((rgb + i)[G]) - (double)((prev_rgb + i)[G]);
    double b = (double)((rgb + i)[B]) - (double)((prev_rgb + i)[B]);
    if (r > 0 && g > 0 && b > 0 && weight > 0)
    {
      diff_[i] += (r + g + b) * weight;
      used++;
    }
    else if (r < 0 && g < 0 && b < 0 && weight < 0)
    {
      diff_[i] += (r + g + b) * weight;
      used++;
    }

    // Is this a new max value?
    if (diff_[i] > max_)
    {
      max_ = diff_[i];
      max_idx_ = i;
    }
  }

  return true;
}

bool LedFinder::CloudDifferenceTracker::isFound(
  const sensor_msgs::msg::PointCloud2& cloud,
  double threshold)
{
  // Returns true only if the max exceeds threshold
  if (max_ < threshold)
  {
    return false;
  }

  // Access point in cloud
  sensor_msgs::PointCloud2ConstIterator<float> point(cloud, "x");
  point += max_idx_;

  // AND the current index is a valid point in the cloud.
  if (std::isnan(point[X]) ||
      std::isnan(point[Y]) ||
      std::isnan(point[Z]))
  {
    return false;
  }

  return true;
}

bool LedFinder::CloudDifferenceTracker::getRefinedCentroid(
  const sensor_msgs::msg::PointCloud2& cloud,
  geometry_msgs::msg::PointStamped& centroid)
{
  // Access point in cloud
  sensor_msgs::PointCloud2ConstIterator<float> iter(cloud, "x");
  const size_t num_points = cloud.data.size() / cloud.point_step;

  // Get initial centroid
  centroid.header = cloud.header;
  centroid.point.x = (iter + max_idx_)[X];
  centroid.point.y = (iter + max_idx_)[Y];
  centroid.point.z = (iter + max_idx_)[Z];

  // Do not accept NANs
  if (std::isnan(centroid.point.x) ||
      std::isnan(centroid.point.y) ||
      std::isnan(centroid.point.z))
  {
    return false;
  }

  // Get a better centroid
  int points = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (size_t i = 0; i < num_points; i++)
  {
    sensor_msgs::PointCloud2ConstIterator<float> point = iter + i;

    // Using highly likely points
    if (diff_[i] > (max_*0.75))
    {
      if (std::isnan(point[X]) || std::isnan(point[Y]) || std::isnan(point[Z]))
      {
        continue;
      }

      double dx = point[X] - centroid.point.x;
      double dy = point[Y] - centroid.point.y;
      double dz = point[Z] - centroid.point.z;

      // That are less than 1cm from the max point
      if ((dx*dx) + (dy*dy) + (dz*dz) < (0.05*0.05))
      {
        sum_x += point[X];
        sum_y += point[Y];
        sum_z += point[Z];
        ++points;
      }
    }
  }

  if (points > 0)
  {
    centroid.point.x = (centroid.point.x + sum_x)/(points+1);
    centroid.point.y = (centroid.point.y + sum_y)/(points+1);
    centroid.point.z = (centroid.point.z + sum_z)/(points+1);
  }

  return true;
}

sensor_msgs::msg::Image LedFinder::CloudDifferenceTracker::getImage()
{
  sensor_msgs::msg::Image image;

  image.height = height_;
  image.width = width_;

  image.encoding = sensor_msgs::image_encodings::BGR8;
  image.step = width_ * 3;

  image.data.resize(width_ * height_ * 3);

  for (size_t i = 0; i < diff_.size(); i++)
  {
    if (diff_[i] > max_ * 0.9)
    {
      image.data[i*3] = 255;
      image.data[i*3 + 1] = 0;
      image.data[i*3 + 2] = 0;
    }
    else if (diff_[i] > 0)
    {
      image.data[i*3] = static_cast<uint8_t>(diff_[i]/2.0);
      image.data[i*3 + 1] = static_cast<uint8_t>(diff_[i]/2.0);
      image.data[i*3 + 2] = static_cast<uint8_t>(diff_[i]/2.0);
    }
    else
    {
      image.data[i*3] = 0;
      image.data[i*3 + 1] = 0;
      image.data[i*3 + 2] = 0;
    }
  }

  return image;
}

}  // namespace robot_calibration

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_calibration::LedFinder, robot_calibration::FeatureFinder)
