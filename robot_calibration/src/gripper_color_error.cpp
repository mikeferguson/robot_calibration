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

GripperColorFinder::GripperColorFinder(ros::NodeHandle& nh) :
  FeatureFinder(nh),
  waiting_(false)
{

  // Setup the action client
  std::string topic_name;
  nh.param<std::string>("action", topic_name, "/gripper_led_action");
  client_.reset(new LedClient(topic_name, true));
  ROS_INFO("Waiting for %s...", topic_name.c_str());
  client_->waitForServer();

  // Setup subscriber
  //nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(/head_camera/rgb/image,
                             1,
                             &GripperColorFinder::cameraCallback,
                             this);

  // Publish where LEDs were seen
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>("led_points", 10);

  // Maximum distance LED can be from expected pose
 // nh.param<double>("max_error", max_error_, 0.1);
  // Maximum relative difference between two LEDs
//  nh.param<double>("max_inconsistency", max_inconsistency_, 0.01);

  // Parameters for detection
  nh.param<double>("threshold", threshold_, 1000.0);
  nh.param<int>("max_iterations", max_iterations_, 50);

  // Should we output debug image/cloud
 // nh.param<bool>("debug", output_debug_, false);

  // Get sensor names
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

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
    *pub = nh.advertise<sensor_msgs::Image>(static_cast<std::string>(led_poses[i]["topic"]), 10);
    tracker_publishers_.push_back(pub);
  }

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }
}


void GripperColorFinder::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  // Lock mutex before updating K
  boost::unique_lock<boost::mutex> lock(mutex_K_);

  float focal_pixels_ = msg->P[0];
  float center_x_ = msg->P[2];
  float center_y_ = msg->P[6];

  if (msg->binning_x == msg->binning_y)
  {
    if (msg->binning_x > 0)
    {
      K_ = (cv::Mat_<double>(3, 3) <<
          focal_pixels_/msg->binning_x, 0.0, center_x_/msg->binning_x,
          0.0, focal_pixels_/msg->binning_x, center_y_/msg->binning_x,
          0.0, 0.0, 1.0);
    }
    else
    {
      K_ = (cv::Mat_<double>(3, 3) <<
          focal_pixels_, 0.0, center_x_,
          0.0, focal_pixels_, center_y_,
          0.0, 0.0, 1.0);
    }
  }
  else
  {
    ROS_ERROR("binning_x is not equal to binning_y");
  }
}


void GripperColorFinder::cameraCallback(const sensor_msgs::ImageConstPtr& image)
{
  if (waiting_)
  {
    image_ = image;
    waiting_ = false;
  }
}

bool GripperColorFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  uint8_t code_idx = -1;

  std::vector<geometry_msgs::PointStamped> rgbd;
  std::vector<geometry_msgs::PointStamped> world;

  robot_calibration_msgs::GripperLedCommandGoal command;
  command.led_code = 0;
  client_->sendGoal(command);
  client_->waitForResult(ros::Duration(10.0)); 

  
  // Initialize difference trackers
  for (size_t i = 0; i < trackers_.size(); ++i)
  {
    trackers_[i].reset(cloud_.height, cloud_.width);
  }

  int cycles = 0;
  while (true)
  {  
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

    geometry_msgs::PointStamped led;
    led.point = trackers_[tracker].point_;
    led.header.frame_id = trackers_[tracker].frame_;
          
    trackers_[tracker].process(cloud_, prev_cloud, led.point, max_error_, weight);

    if (++cycles > max_iterations_)
    {
      ROS_ERROR("Failed to find features before using maximum iterations.");
      return false;
    }

    prev_cloud = cloud_;
   
    // Export results
  msg->observations.resize(2);
  msg->observations[0].sensor_name = camera_sensor_name_;
  msg->observations[1].sensor_name = chain_sensor_name_;

  for (size_t t = 0; t < trackers_.size(); ++t)
  {
    geometry_msgs::PointStamped rgbd_pt;
    geometry_msgs::PointStamped world_pt;
    
    rgbd_pt.x  = trackers_[t];
    




} 

}  
