// Author: Jack Center
// TOOD: attempt to launch as is

#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/CaptureConfig.h>

#include <robot_calibration/capture/chain_manager.h>
#include <robot_calibration/capture/feature_finder_loader.h>

#include <camera_calibration_parsers/parse.h>
#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/camera_info.h>
#include <robot_calibration/load_bag.h>

#include <boost/foreach.hpp>  // for rosbag iterator

#include <robot_calibration/capture_features.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv,"robot_calibration");
  ros::NodeHandle nh("~");

  check_parameters(nh);

  // The calibration data
  std_msgs::String description_msg;
  std::vector<robot_calibration_msgs::CalibrationData> data;
  bool capture_successful;

  // Setup capture

  // Auto capture mode
  if (nh.param<bool>("auto_calibration_mode", false))
  {
    capture_successful = run_automatic_capture(nh, data);
  }

  // Manual capture mode
  else
  {
    capture_successful = run_manual_capture(nh, data);
  }

  // Save feature data
  if (!save_data("file_name"))
  {
    ROS_ERROR_STREAM("unable to save features");
  };  

  return 0;
}

void check_parameters(ros::NodeHandle& nh)
{
  std::string auto_capture_mode_name = "auto_capture_mode";
  std::string verbose_mode_name = "verbose";

  // Checks that parameter exits and is a bool
  if (!nh.hasParam(auto_capture_mode_name))
  {
    ROS_ERROR_STREAM("calibration capture mode not set. Defaulting to manual mode. The value can be set in the launch file.");
    nh.setParam(auto_capture_mode_name, false);
  }

  if (!nh.hasParam(verbose_mode_name))
  {
    ROS_ERROR_STREAM("verbose output not set. Defaulting to standard output. The value can be set in the launch file.");
    nh.setParam(verbose_mode_name, false);
  }

}

bool run_automatic_capture(ros::NodeHandle& nh, std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  std::vector<sensor_msgs::JointState> poses = load_calibration_poses();
  start_bag("/msg/name");
  
  // Loop - for each loaded pose
  for (auto pose : poses) {
    /* TODO:
        1. Move to the pose
        2. Wait to settle
        3. Capture the calibration data
    */
    move_to_position();
    capture_calibration_data();
  }

  stop_bag("/msg/name");

  return true;
}

bool run_manual_capture(ros::NodeHandle& nh, std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  start_bag("/msg/name");
  start_bag("/msg/name");

  bool capture_complete = false;

  // Loop - while poses are still being captured
  while (!capture_complete) 
  {
    // Manual calibration, wait for keypress
    ROS_INFO("Press key when arm is ready... (type 'done' to finish capture)");
    std::string user_input;
    std::getline(std::cin, user_input);

    if (user_input.compare("done") == 0)
      break;

    if (!ros::ok())
      break;

    capture_calibration_data();
  }

  stop_bag("/msg/name");
  stop_bag("/msg/name");

  return true; 
}

// TODO:
bool save_data(std::string file_name) 
{
  return true;
}

// TODO: 
bool start_bag(std::string msg_name)
{
  return true;
}

// TODO: 
bool stop_bag(std::string msg_name)
{
  return true;
}

// TODO:
std::vector<sensor_msgs::JointState> load_calibration_poses()
{
  std::vector<sensor_msgs::JointState> val {};
  return val;
}

// TODO:
bool move_to_position()
{
  return true;
}

// TODO:
bool wait_for_manual_move() {
  return true;
}

// TODO:
bool capture_pose()
{
  return true;
}

// TODO:
bool capture_calibration_data()
{
  // TODO: get full CalibrationData message
  return true;
}
