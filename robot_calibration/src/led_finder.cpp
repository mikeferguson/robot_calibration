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

#include <math.h>
#include <robot_calibration/capture/led_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>

namespace robot_calibration
{

double distancePoints(
  const geometry_msgs::Point p1,
  const geometry_msgs::Point p2)
{
  return std::sqrt((p1.x-p2.x) * (p1.x-p2.x) +
                   (p1.y-p2.y) * (p1.y-p2.y) +
                   (p1.z-p2.z) * (p1.z-p2.z));
}

LedFinder::LedFinder(ros::NodeHandle & n) :
  FeatureFinder(n),
  waiting_(false)
{
  ros::NodeHandle nh(n, "led_finder");

  // Setup the action client
  std::string topic_name;
  nh.param<std::string>("action", topic_name, "/gripper_led_action");
  client_.reset(new LedClient(topic_name, true));
  ROS_INFO("Waiting for %s...", topic_name.c_str());
  client_->waitForServer();

  // Setup subscriber
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = n.subscribe(topic_name,
                            1,
                            &LedFinder::cameraCallback,
                            this);

  // Publish where LEDs were seen
  publisher_ = n.advertise<sensor_msgs::PointCloud2>("led_points", 10);

  // Maximum distance LED can be from expected pose
  nh.param<double>("max_error", max_error_, 0.1);
  // Maximum relative difference between two LEDs
  nh.param<double>("max_inconsistency", max_inconsistency_, 0.01);

  // Parameters for detection
  nh.param<double>("threshold", threshold_, 1000.0);
  nh.param<int>("max_iterations", max_iterations_, 50);

  // Should we output debug image/cloud
  nh.param<bool>("debug", output_debug_, false);

  // Parameters for LEDs themselves
  std::string gripper_led_frame;
  nh.param<std::string>("gripper_led_frame", gripper_led_frame, "wrist_roll_link");
  XmlRpc::XmlRpcValue led_poses;
  nh.getParam("poses", led_poses);
  ROS_ASSERT(led_poses.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // Each LED has a code, and pose in the gripper_led_frame
  for (int i = 0; i < led_poses.size(); ++i)
  {
    codes_.push_back(static_cast<int>(led_poses[i]["code"]));
    codes_.push_back(0);  // assumes "0" is code for "OFF"

    double x, y, z;
    x = static_cast<double>(led_poses[i]["x"]);
    y = static_cast<double>(led_poses[i]["y"]);
    z = static_cast<double>(led_poses[i]["z"]);
    trackers_.push_back(CloudDifferenceTracker(gripper_led_frame, x, y, z));

    // Publisher
    boost::shared_ptr<ros::Publisher> pub(new ros::Publisher);
    *pub = n.advertise<sensor_msgs::Image>(static_cast<std::string>(led_poses[i]["topic"]), 10);
    tracker_publishers_.push_back(pub);
  }
}

void LedFinder::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if (waiting_)
  {
    cloud_ptr_ = cloud;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout.
bool LedFinder::waitForCloud()
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

bool LedFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  uint8_t code_idx = -1;

  std::vector<geometry_msgs::PointStamped> rgbd;
  std::vector<geometry_msgs::PointStamped> world;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  robot_calibration_msgs::GripperLedCommandGoal command;
  command.led_code = 0;
  client_->sendGoal(command);
  client_->waitForResult(ros::Duration(10.0));

  // Get initial cloud
  if (!waitForCloud())
  {
    return false;
  }
  *prev_cloud = *cloud_ptr_;

  // Initialize difference trackers
  for (size_t i = 0; i < trackers_.size(); ++i)
  {
    trackers_[i].reset(cloud_ptr_->height, cloud_ptr_->width);
  }

  int cycles = 0;
  while (true)
  {
    // Toggle LED to next state
    code_idx = (code_idx + 1) % 8;
    command.led_code = codes_[code_idx];
    client_->sendGoal(command);
    client_->waitForResult(ros::Duration(10.0));

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
      done &= trackers_[t].isFound(cloud_ptr_, threshold_);
    }
    // We want to break only if the LED is off, so that pixel is not washed out
    if (done && (weight == -1))
    {
      break;
    }

    // Get expected pose of LED in the cloud frame
    geometry_msgs::PointStamped led;
    led.point = trackers_[tracker].point;
    led.header.frame_id = trackers_[tracker].frame_;
    try
    {
      listener_.transformPoint(cloud_ptr_->header.frame_id, ros::Time(0), led,
                               led.header.frame_id, led);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Failed to transform feature to " << cloud_ptr_->header.frame_id);
      return false;
    }

    // Update the tracker
    trackers_[tracker].process(cloud_ptr_, prev_cloud, led.point, max_error_, weight);

    if (++cycles > max_iterations_)
    {
      return false;
    }

    *prev_cloud = *cloud_ptr_;

    // Publish state of each tracker
    for (size_t i = 0; i < trackers_.size(); i++)
    {
      sensor_msgs::Image image = trackers_[i].getImage();
      tracker_publishers_[i]->publish(image);
    }
  }

  // Create PointCloud2 to publish
  sensor_msgs::PointCloud2 cloud;
  cloud.width = 0;
  cloud.height = 0;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = cloud_ptr_->header.frame_id;
  sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(4);
  sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

  // Export results
  msg->observations.resize(2);
  msg->observations[0].sensor_name = "camera";  // TODO: parameterize
  msg->observations[1].sensor_name = "arm";     // TODO: parameterize
  for (size_t t = 0; t < trackers_.size(); ++t)
  {
    geometry_msgs::PointStamped rgbd_pt;
    geometry_msgs::PointStamped world_pt;

    // Get point
    if (!trackers_[t].getRefinedCentroid(cloud_ptr_, rgbd_pt))
    {
      ROS_ERROR_STREAM("No centroid for feature " << t);
      return false;
    }

    // Check that point is close enough to expected pose
    try
    {
      listener_.transformPoint(trackers_[t].frame_, ros::Time(0), rgbd_pt,
                               rgbd_pt.header.frame_id, world_pt);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR_STREAM("Failed to transform feature to " << trackers_[t].frame_);
      return false;
    }
    double distance = distancePoints(world_pt.point, trackers_[t].point);
    if (distance > max_error_)
    {
      ROS_ERROR_STREAM("Feature was too far away from expected pose in " << trackers_[t].frame_ << ": " << distance);
      return false;
    }

    // Check that points are consistent with one another
    for (size_t t2 = 0; t2 < t; ++t2)
    {
      double expected = distancePoints(trackers_[t2].point, trackers_[t].point);
      double actual = distancePoints(msg->observations[0].features[t2].point, rgbd_pt.point);
      if (fabs(expected-actual) > max_inconsistency_)
      {
        ROS_ERROR_STREAM("Features not internally consistent: " << expected << " " << actual);
        return false;
      }
    }

    // Push back observation
    msg->observations[0].features.push_back(rgbd_pt);

    // Visualize
    iter_cloud[0] = rgbd_pt.point.x;
    iter_cloud[1] = rgbd_pt.point.y;
    iter_cloud[2] = rgbd_pt.point.z;
    ++iter_cloud;

    // Push back expected location of point on robot
    world_pt.header.frame_id = trackers_[t].frame_;
    world_pt.point = trackers_[t].point;
    msg->observations[1].features.push_back(world_pt);
  }

  // Final check that all points are valid
  if (msg->observations[0].features.size() != trackers_.size())
  {
    return false;
  }

  // Add debug cloud to message
  if (output_debug_)
  {
    pcl::toROSMsg(*cloud_ptr_, msg->observations[0].cloud);
  }

  // Publish results
  publisher_.publish(cloud);

  return true;
}

LedFinder::CloudDifferenceTracker::CloudDifferenceTracker(
  std::string frame, double x, double y, double z) :
    frame_(frame)
{
  point.x = x;
  point.y = y;
  point.z = z;
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev,
  geometry_msgs::Point& led_point,
  double max_distance,
  double weight)
{
  if (cloud->size() != diff_.size())
  {
    std::cerr << "Cloud size has changed." << std::endl;
    return false;
  }

  // We want to compare each point to the expected LED pose,
  // but when the LED is on, the points will be NAN,
  // fall back on most recent distance for these points
  double last_distance = 1000.0;

  // Update each point in the tracker
  for (size_t i = 0; i < cloud->size(); i++)
  {
    // If within range of LED pose...
    geometry_msgs::Point p;
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    double distance = distancePoints(p, led_point);

    if (std::isfinite(distance))
    {
      last_distance = distance;
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
    double r = (double)(cloud->points[i].r) - (double)(prev->points[i].r);
    double g = (double)(cloud->points[i].g) - (double)(prev->points[i].g);
    double b = (double)(cloud->points[i].b) - (double)(prev->points[i].b);
    if (r > 0 && g > 0 && b > 0 && weight > 0)
    {
      diff_[i] += (r + g + b) * weight;
    }
    else if (r < 0 && g < 0 && b < 0 && weight < 0)
    {
      diff_[i] += (r + g + b) * weight;
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
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  double threshold)
{
  // Returns true only if the max exceeds threshold
  if (max_ < threshold)
  {
    return false;
  }

  // AND the current index is a valid point in the cloud.
  if (isnan(cloud->points[max_idx_].x) ||
      isnan(cloud->points[max_idx_].x) ||
      isnan(cloud->points[max_idx_].x))
  {
    return false;
  }

  return true;
}

bool LedFinder::CloudDifferenceTracker::getRefinedCentroid(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  geometry_msgs::PointStamped& point)
{
  // Get initial centroid
  geometry_msgs::PointStamped p;
  point.point.x = cloud->points[max_idx_].x;
  point.point.y = cloud->points[max_idx_].y;
  point.point.z = cloud->points[max_idx_].z;

  // Do not accept NANs
  if (isnan(point.point.x) ||
      isnan(point.point.y) ||
      isnan(point.point.z))
  {
    return false;
  }

  // Get a better centroid
  int points = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (isnan(cloud->points[i].x) ||
        isnan(cloud->points[i].y) ||
        isnan(cloud->points[i].z))
    {
      continue;
    }

    // Using highly likely points
    if (diff_[i] > (max_*0.75))
    {
      double dx = cloud->points[i].x - p.point.x;
      double dy = cloud->points[i].y - p.point.y;
      double dz = cloud->points[i].z - p.point.z;

      // That are less than 5mm from the max point
      if ( (dx*dx) + (dy*dy) + (dz*dz) < 0.22 )
      {
        sum_x += cloud->points[i].x;
        sum_y += cloud->points[i].y;
        sum_z += cloud->points[i].z;
        ++points;
      }
    }
  }

  if (points > 0)
  {
    point.point.x = sum_x/points;
    point.point.y = sum_y/points;
    point.point.z = sum_z/points;
  }

  /* Fill in the headers */
  point.header.seq = cloud->header.seq;
  point.header.frame_id = cloud->header.frame_id;
  point.header.stamp.fromNSec(cloud->header.stamp * 1e3);  // from pcl_conversion

  return true;
}

sensor_msgs::Image LedFinder::CloudDifferenceTracker::getImage()
{
  sensor_msgs::Image image;

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
