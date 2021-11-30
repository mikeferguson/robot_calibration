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

  if (!check_parameters(nh))
  {
    ROS_FATAL("Parameters couldn't be properly loaded");
    return -1;
  }

  // Get the robot_description and republish it
  ros::Publisher urdf_pub = nh.advertise<std_msgs::String>("/robot_description", 1, true);  // latched
  std_msgs::String description_msg;
  if (!nh.getParam("/robot_description", description_msg.data))
  {
    ROS_FATAL("robot_description not set!");
    return -1;
  }
  urdf_pub.publish(description_msg);

  // The calibration data
  std::vector<robot_calibration_msgs::CalibrationData> data;
  bool capture_successful;

  // Auto capture mode
  if (nh.param<bool>("auto_calibration_mode", false))
  {
    ROS_INFO("Using automatic calibration mode...");
    capture_successful = run_automatic_capture(nh, data);
  }

  // Manual capture mode
  else
  {
    ROS_INFO("Using manual calibration mode...");
    capture_successful = run_manual_capture(nh, data);
  }

  ROS_INFO("Done capturing samples");

  return 0;
}

bool check_parameters(ros::NodeHandle& nh)
{
  bool success = true;
  std::string auto_capture_mode_name = "auto_capture_mode";
  std::string verbose_mode_name = "verbose";

  std::string bag_filename = "bag_filename";
  std::string feature_finder_name = "feature_finder";

  // Checks that parameter exists and is a bool
  if (!nh.hasParam(auto_capture_mode_name))
  {
    ROS_ERROR_STREAM("calibration capture mode not set. Defaulting to manual mode. The value can be set in the launch file.");
    nh.setParam(auto_capture_mode_name, false);
  }

  // Checks that parameter exists and is a bool
  if (!nh.hasParam(verbose_mode_name))
  {
    ROS_ERROR_STREAM("verbose output not set. Defaulting to standard output. The value can be set in the launch file.");
    nh.setParam(verbose_mode_name, false);
  }

  if (!nh.hasParam(bag_filename))
  {
    ROS_ERROR_STREAM("bag filename parameter not set. Setting default location");
    std::string bag_filename_default = get_absolute_directory("/rosbags/default.bag");
    nh.setParam(bag_filename, bag_filename_default);
  }

  if (!nh.hasParam(feature_finder_name)){
    ROS_ERROR_STREAM("feature parameter not set. Defaulting to checkerboard finder. The value can be set in the launch file.");
    nh.setParam(feature_finder_name, "checkerboard_finder");
  }

  ROS_INFO_STREAM("parameters set");
  return success;
}


std::string get_absolute_directory(std::string local_dir)
{
  std::string home_dir = getenv("HOME");
  std::string absolute_path = home_dir + local_dir;
  return absolute_path;
}


bool run_automatic_capture(ros::NodeHandle& nh, std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  // TODO
  std::string feature; 
  robot_calibration::FeatureFinderMap finders;                      // The available feature finders
  robot_calibration::FeatureFinderLoader feature_finder_loader;     // Helper to load the feature finders
  robot_calibration::ChainManager chain_manager(nh);
  robot_calibration_msgs::CalibrationData msg;
  bool capture_complete = false;

  if (!nh.getParam("feature_finder", feature))
  {
    ROS_FATAL("Unable to load the selected feature");
  }
  
  if (!feature_finder_loader.load(nh, finders))
  {
    ROS_FATAL("Unable to load feature finders");
  }

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);

  std::vector<robot_calibration_msgs::CaptureConfig> poses;
// TODO: get a filename
  load_calibration_poses("test", poses);
  
  // Loop - for each loaded pose
  for (auto pose : poses) {
    /* TODO:
        1. Move to the pose
        2. Wait to settle
        3. Capture the calibration data
    */
    capture_calibration_data(chain_manager, finders, feature, msg);
    data.emplace_back(msg);
  }

  return true;
}

bool run_manual_capture(ros::NodeHandle& nh, std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  std::string feature;                                              // Name of the feature finder (ie checkerboard_finder)
  robot_calibration::FeatureFinderMap finders;                      // The available feature finders
  robot_calibration::FeatureFinderLoader feature_finder_loader;     // Helper to load the feature finders
  robot_calibration::ChainManager chain_manager(nh, 0.1);           // Manages kinematic chains
  robot_calibration_msgs::CalibrationData msg;                      // Data message place holder
  std::string bag_filename;                                         // Absolute path to location to save the bagfile
  bool capture_complete = false;

  if (!nh.getParam("feature_finder", feature))
  {
    ROS_FATAL("Unable to load the selected feature finder");
  }

// TODO: why won't this unload?
  if (!feature_finder_loader.load(nh, finders))
  {
    ROS_FATAL("Unable to load feature finders");
  }

  if (!nh.getParam("bag_filename", bag_filename))
  {
    ROS_FATAL("Unable to create bag file");
  }

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);
  rosbag::Bag bag(bag_filename, rosbag::bagmode::Write);

  // Loop - while poses are still being captured
  while (!capture_complete && ros::ok())
  {
    // Manual calibration, wait for keypress
    ROS_INFO("Press key when arm is ready... (type 'done' to finish capture)");
    std::string user_input;
    std::getline(std::cin, user_input);

    // User has ended the capture
    if (user_input.compare("done") == 0)
    {
      capture_complete = true;
    }

    // Capture was successful
    else if (capture_calibration_data(chain_manager, finders, feature, msg))
    {
      ROS_INFO("Captured pose %lu", data.size());
      chain_manager.getState(& msg.joint_states);
      pub.publish(msg);
      data.emplace_back(msg);
      bag.write("calibration_data", ros::Time::now(), msg);
    }

    // Capture was unsuccessful
    else
    {
      ROS_WARN("Failed to capture sample %lu.", data.size());
    }
  }

  bag.close();

  // TODO: should have this return something more useful
  return capture_complete; 
}


// TODO:
bool load_calibration_poses(std::string filename, std::vector<robot_calibration_msgs::CaptureConfig>& poses)
{
  bool success = true;
  rosbag::Bag bag;

  if (!open_bag(filename, bag))
  {
    success = false;
  }

  else 
  {
    rosbag::View data_view(bag, rosbag::TopicQuery("calibration_joint_states"));
    BOOST_FOREACH (rosbag::MessageInstance const m, data_view)
    {
      robot_calibration_msgs::CaptureConfig::ConstPtr msg = m.instantiate<robot_calibration_msgs::CaptureConfig>();
      poses.emplace_back(*msg);
    }
  }

  return success;
}


bool open_bag(std::string filename, rosbag::Bag& bag)
{
  bool success = true;

  ROS_INFO_STREAM("Opening " << filename);

  try
  {
    bag.open(filename, rosbag::bagmode::Read);
  }

  catch (rosbag::BagException&)
  {
    ROS_FATAL_STREAM("Cannot open " << filename);
    success = false;
  }

  return success;
}


// TODO:
bool move_to_position()
{
  return true;
}


bool capture_calibration_data(robot_calibration::ChainManager& chain_manager, 
                              robot_calibration::FeatureFinderMap& finders, 
                              std::string& feature,
                              robot_calibration_msgs::CalibrationData& msg)
{
  bool feature_found = true;

  // Wait for joints to settle
  chain_manager.waitToSettle();

  // Make sure sensor data is up to date after settling
  ros::Duration(0.1).sleep();

  // Capture the feature
  if (!finders[feature]->find(& msg))
  {
    ROS_WARN("%s failed to capture features.", feature.c_str());
    feature_found = false;
  }

  return feature_found;
}
