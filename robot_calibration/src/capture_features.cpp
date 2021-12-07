// Author: Jack Center

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/String.h>

#include <robot_calibration/capture_features.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv,"robot_calibration");
  ros::NodeHandle nh("~");

  if (!capture_features::check_parameters(nh))
  {
    ROS_FATAL("parameter checking failed.");
    return -1;
  }

  std::string bag_filename;                                 // Absolute path to location to save the bagfile
  nh.getParam("bag_filename", bag_filename);  
  rosbag::Bag bag(bag_filename, rosbag::bagmode::Write);    // rosbag to store calibration data

  std::string feature;
  nh.getParam("feature_finder", feature);                   // The feature to search for during caputre

  // Set up the calibration
  std_msgs::String description_msg = capture_features::republish_robot_description(nh);
  bag.write("/robot_description", ros::Time::now(), description_msg);

  bool capture_successful = false;

  // Auto capture mode
  if (nh.param<bool>("auto_calibration_mode", false))
  {
    ROS_INFO("Using automatic calibration mode...");
    capture_successful = capture_features::run_automatic_capture(nh, feature, &bag);
  }

  // Manual capture mode
  else
  {
    ROS_INFO("Using manual calibration mode...");
    capture_successful = capture_features::run_manual_capture(nh, feature, &bag);
  }

  bag.close();
  ROS_INFO("Done capturing samples");

  return 0;
}
