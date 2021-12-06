#include <optional>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/String.h>

#include <boost/foreach.hpp>  // for rosbag iterator

#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/CaptureConfig.h>

#include <robot_calibration/capture/chain_manager.h>
#include <robot_calibration/capture/feature_finder_loader.h>
#include <robot_calibration/capture_features.h>


namespace capture_features {

bool check_parameters(const ros::NodeHandle& nh)
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


std::string get_absolute_directory(const std::string& local_dir)
{
  std::string home_dir = getenv("HOME");
  std::string absolute_path = home_dir + local_dir;
  return absolute_path;
}


std::optional<robot_calibration::FeatureFinderMap> get_feature_finders(ros::NodeHandle& nh)
{
  robot_calibration::FeatureFinderLoader feature_finder_loader;     // Helper to load the feature finders
  robot_calibration::FeatureFinderMap finders;

// TODO: why won't this unload?
  if (!feature_finder_loader.load(nh, finders))
  {
    ROS_FATAL("Unable to load feature finders");
    return {};
  }

  return finders;
}


std_msgs::String republish_robot_description(ros::NodeHandle& nh)
{
  std_msgs::String description_msg;
  nh.getParam("/robot_description", description_msg.data);
  ros::Publisher urdf_pub = nh.advertise<std_msgs::String>("/robot_description", 1, true);  // latched
  urdf_pub.publish(description_msg);

  return description_msg;
}


bool run_automatic_capture(ros::NodeHandle& nh,
                        const std::string& feature,
                        rosbag::Bag& bag)
{
  // TODO
  robot_calibration::ChainManager chain_manager(nh, 0.1);     // Manages kinematic chains
  
  auto finders = get_feature_finders(nh);        // Holds the available feature finders  
  if (!finders.has_value())
  { 
    ROS_FATAL("the loaded feature finder map is empty.");
    return false;    
  }

  robot_calibration_msgs::CalibrationData msg;
  bool capture_complete = false;

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);

// TODO: get a filename
  std::vector<robot_calibration_msgs::CaptureConfig> poses = load_calibration_poses("test");
  
  // Loop - for each loaded pose
  for (auto pose : poses) {
    /* TODO:
        1. Move to the pose
        2. Wait to settle
        3. Capture the calibration data
    */
    auto msg = capture_calibration_data(chain_manager, *finders, feature);
  }

  return true;
}

bool run_manual_capture(ros::NodeHandle& nh,
                        const std::string& feature,
                        rosbag::Bag& bag)
{
  ROS_DEBUG("Running manual capture.");
  bool capture_complete = false;

  robot_calibration::ChainManager chain_manager(nh, 0.0);     // Manages kinematic chains

  auto finders = get_feature_finders(nh);        // Holds the available feature finders  
  if (!finders.has_value())
  { 
    ROS_WARN("feature finders failed to load");
    return false;
  }

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);
  int captured_poses = 0;
  
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
      continue;
    }

    auto msg = capture_calibration_data(chain_manager, *finders, feature);

    // Capture unsuccessful
    if(!msg.has_value())
    {
      ROS_WARN("Failed to capture sample");
    }

    // Capture successful
    else
    {

      captured_poses++;
      ROS_INFO("Captured pose %i", captured_poses);
      chain_manager.getState(& msg->joint_states);
      pub.publish(*msg);
      bag.write("/calibration_data", ros::Time::now(), *msg);
    }
  }

  return capture_complete; 
}


std::vector<robot_calibration_msgs::CaptureConfig> load_calibration_poses(const std::string& filename)
{
  rosbag::Bag bag;
  std::vector<robot_calibration_msgs::CaptureConfig> poses;

  bag = open_bag(filename);

  rosbag::View data_view(bag, rosbag::TopicQuery("calibration_joint_states"));
  BOOST_FOREACH (rosbag::MessageInstance const m, data_view)
  {
    robot_calibration_msgs::CaptureConfig::ConstPtr msg = m.instantiate<robot_calibration_msgs::CaptureConfig>();
    poses.emplace_back(*msg);
  }

  return poses;
}


rosbag::Bag open_bag(const std::string& filename)
{
  ROS_INFO_STREAM("Opening " << filename);

  rosbag::Bag bag;

  try
  {
    bag.open(filename, rosbag::bagmode::Read);
  }

  catch (rosbag::BagException&)
  {
    ROS_FATAL_STREAM("Cannot open " << filename);
  }

  return bag;
}


// TODO:
bool move_to_position()
{
  return true;
}


std::optional<robot_calibration_msgs::CalibrationData> capture_calibration_data(robot_calibration::ChainManager& chain_manager, 
                                                                                robot_calibration::FeatureFinderMap& finders, 
                                                                                const std::string& feature)
{
  ROS_DEBUG("capturing calibration data from pose.");

  // Wait for joints to settle
  chain_manager.waitToSettle();

  // Make sure sensor data is up to date after settling
  ros::Duration(0.1).sleep();

  // Capture the feature
  robot_calibration_msgs::CalibrationData msg; 
  if (!finders[feature]->find(& msg))
  {
    ROS_WARN("%s failed to capture features.", feature.c_str());
    return {};
  }

  return msg;
}

} // namespacer capture_features