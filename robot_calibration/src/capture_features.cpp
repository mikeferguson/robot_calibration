// Author: Jack Center

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
    return -1;

  // Set up the calibration
// TODO: make timeout a define
  robot_calibration::ChainManager chain_manager(nh, 0.1);     // Manages kinematic chains

  robot_calibration::FeatureFinderMap finders;               // Holds the available feature finders  
  if (!get_feature_finders(nh, finders)) 
    return -1;    

  std::string feature;
  nh.getParam("feature_finder", feature);                    // The feature to search for during caputre

  std::string bag_filename;                                   // Absolute path to location to save the bagfile
  nh.getParam("bag_filename", bag_filename);  
  rosbag::Bag bag(bag_filename, rosbag::bagmode::Write);

  // Get the robot_description and republish it
// TODO: describe why this is needed.
  std_msgs::String description_msg;
  nh.getParam("/robot_description", description_msg.data);
  ros::Publisher urdf_pub = nh.advertise<std_msgs::String>("/robot_description", 1, true);  // latched
  urdf_pub.publish(description_msg);
  bag.write("/robot_description", ros::Time::now(), description_msg);

  bool capture_successful;

  // Auto capture mode
  if (nh.param<bool>("auto_calibration_mode", false))
  {
    ROS_INFO("Using automatic calibration mode...");
    capture_successful = run_automatic_capture(nh, &chain_manager, finders, feature, bag);
  }

  // Manual capture mode
  else
  {
    ROS_INFO("Using manual calibration mode...");
    capture_successful = run_manual_capture(nh, &chain_manager, finders, feature, bag);
  }

  bag.close();
  ROS_INFO("Done capturing samples");

  return 0;
}

bool check_parameters(ros::NodeHandle& nh)
{
  bool success = true;
  std::string auto_capture_mode_name = "auto_capture_mode";

  std::string bag_filename = "bag_filename";
  std::string feature_finder_name = "feature_finder";

  // Checks that parameter exists and is a bool
  if (!nh.hasParam(auto_capture_mode_name))
  {
    ROS_ERROR_STREAM("calibration capture mode not set. Defaulting to manual mode. The value can be set in the launch file.");
    nh.setParam(auto_capture_mode_name, false);
  }

  if (!nh.hasParam(bag_filename))
  {
    ROS_ERROR_STREAM("bag filename parameter not set. Setting default location");
// TODO: fix this to be a better default
    std::string bag_filename_default = get_absolute_directory("/rosbags/default.bag");
    nh.setParam(bag_filename, bag_filename_default);
  }

  if (!nh.hasParam(feature_finder_name)){
    ROS_ERROR_STREAM("feature parameter not set. Defaulting to checkerboard finder. The value can be set in the launch file.");
    nh.setParam(feature_finder_name, "checkerboard_finder");
  }

  if (!nh.hasParam("/robot_description"))
  {
    ROS_FATAL("robot_description not set. Exiting the program");
    success = false;
  }

  if (success)
  {
    ROS_INFO_STREAM("parameters successfully set");
  }

  else{
    ROS_FATAL("unable to set the parameters");
  }

  return success;
}


std::string get_absolute_directory(std::string local_dir)
{
  std::string home_dir = getenv("HOME");
  std::string absolute_path = home_dir + local_dir;
  return absolute_path;
}


bool get_feature_finders(ros::NodeHandle& nh, robot_calibration::FeatureFinderMap& finders)
{
  bool success = true;
  robot_calibration::FeatureFinderLoader feature_finder_loader;     // Helper to load the feature finders

// TODO: why won't this unload?
  if (!feature_finder_loader.load(nh, finders))
  {
    ROS_FATAL("Unable to load feature finders");
    success = false;
  }

  return success;
}


bool run_automatic_capture(ros::NodeHandle& nh, 
                        robot_calibration::ChainManager* chain_manager, 
                        robot_calibration::FeatureFinderMap& finders,
                        std::string& feature,
                        rosbag::Bag& bag)
{
  // TODO
  robot_calibration_msgs::CalibrationData msg;
  bool capture_complete = false;

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
  }

  return true;
}

bool run_manual_capture(ros::NodeHandle& nh, 
                        robot_calibration::ChainManager* chain_manager, 
                        robot_calibration::FeatureFinderMap& finders,
                        std::string& feature,
                        rosbag::Bag& bag)
{
  robot_calibration_msgs::CalibrationData msg;                      // Data message place holder
  int captured_poses;

  bool capture_complete = false;

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);
  

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
      captured_poses++;
      ROS_INFO("Captured pose %i", captured_poses);
      chain_manager->getState(& msg.joint_states);
      pub.publish(msg);
      bag.write("/calibration_data", ros::Time::now(), msg);
    }

    // Capture was unsuccessful
    else
    {
      ROS_WARN("Failed to capture sample");
    }
  }

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


bool capture_calibration_data(robot_calibration::ChainManager* chain_manager, 
                              robot_calibration::FeatureFinderMap& finders, 
                              std::string& feature,
                              robot_calibration_msgs::CalibrationData& msg)
{
  bool feature_found = true;

  // Wait for joints to settle
  chain_manager->waitToSettle();

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
