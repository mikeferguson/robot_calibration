
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <boost/foreach.hpp>  // for rosbag iterator

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/optimize_parameters.h>


// TODO: make topic names not hardcoded 
void publish_messages(ros::NodeHandle&, std::vector<robot_calibration_msgs::CalibrationData>&);

int main(int argc, char** argv)
{
  ros::init(argc, argv,"robot_calibration");
  ros::NodeHandle nh("~");

  // Parameters
  std::string bag_filename;                                   // Absolute path to location to save the bagfile
  bool verbose;                                               // Level of data to output to the screen

  if(!check_parameters(nh))
    return -1;

  nh.getParam("bag_filename", bag_filename); 
  nh.getParam("verbose", verbose);

  // Load data
  ROS_INFO("Loading optimization data");
  std_msgs::String description_msg;
  std::vector<robot_calibration_msgs::CalibrationData> data;

  if(!load_data(bag_filename, description_msg, data))
    return -1;

publish_messages(nh, data);
  // Run optimization
  ROS_INFO("Running optimization");
  robot_calibration::OptimizationParams params;
  robot_calibration::Optimizer opt(description_msg.data);

  params.LoadFromROS(nh);
  opt.optimize(params, data, verbose);

  // Output optimization to screen
  if (verbose)
  {
    std::cout << "Parameter Offsets:" << std::endl;
    std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
  }

  ROS_INFO("Done calibrating");

  return 0;
}


bool check_parameters(ros::NodeHandle& nh)
{
  bool success = true;
  std::string verbose_mode_name = "verbose";
  std::string bag_filename = "bag_filename";

  // Checks that parameter exists and is a bool
  if (!nh.hasParam(verbose_mode_name))
  {
    ROS_ERROR_STREAM("verbose output not set. Defaulting to standard output. The value can be set in the launch file.");
    nh.setParam(verbose_mode_name, false);
  }

  if (!nh.hasParam(bag_filename))
  {
    ROS_ERROR_STREAM("bag filename parameter not set. Setting default location");
// TODO: fix this to be a better default
    std::string bag_filename_default = get_absolute_directory("/rosbags/default.bag");
    nh.setParam(bag_filename, bag_filename_default);
  }

  if (success)
  {
    ROS_DEBUG_STREAM("parameters successfully set");
  }

  else{
    ROS_FATAL("unable to set the parameters");
  }

  return success;
}


std::string get_absolute_directory(std::string local_dir)
{
  ROS_DEBUG_STREAM("getting the absolute path to " << local_dir);

  std::string home_dir = getenv("HOME");
  std::string absolute_path = home_dir + local_dir;

  return absolute_path;
}


bool load_data(std::string& bag_filename, 
                           std_msgs::String& description_msg,
                           std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  ROS_DEBUG("starting to load data.");

  rosbag::Bag bag;
  if(!open_rosbag(bag, bag_filename))
    return false;

  if(!load_robot_description(bag, description_msg))
    return false;

  if(!load_calibration_data(bag, data))
    return false;

  bag.close();

  return true;
}


bool open_rosbag(rosbag::Bag& bag, std::string& bag_filename)
{
  ROS_DEBUG_STREAM("openning rosbag: " << bag_filename);
  bool success = true;

  try
  { 
    bag.open(bag_filename, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException&)
  {
    ROS_FATAL_STREAM("Could not open calibration rosbag: " << bag_filename);
    success = false;
  }

  return success;
}


bool load_robot_description(rosbag::Bag& bag, std_msgs::String& description_msg)
{
  ROS_DEBUG("loading robot description.");
  bool success;

  rosbag::View bag_view(bag, rosbag::TopicQuery("/robot_description"));

  if (bag_view.size() < 1)
  {
    ROS_FATAL("'/robot_description' topic not found in rosbag file.");
    success = false;
  }

// TODO: what if there are more than 1 found?
  else
  {
    std_msgs::String::ConstPtr msg = bag_view.begin()->instantiate<std_msgs::String>();
    description_msg = *msg;
    success = true;
  }

  return success;
}


bool load_calibration_data(rosbag::Bag& bag, std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  ROS_DEBUG("loading calibration data.");
  bool success;

  rosbag::View bag_view(bag, rosbag::TopicQuery("/calibration_data"));

  if (bag_view.size() < 1)
  {
    ROS_FATAL("'/calibration_data' topic not found in rosbag file.");
    success = false;
  }

  BOOST_FOREACH (rosbag::MessageInstance const m, bag_view)
  {
    robot_calibration_msgs::CalibrationData::ConstPtr msg = m.instantiate<robot_calibration_msgs::CalibrationData>();
    data.push_back(*msg);
    success = true;
  }

  return success;
}


void publish_messages(ros::NodeHandle& nh, std::vector<robot_calibration_msgs::CalibrationData>& data) {

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);

  for (auto msg : data)
  {
    pub.publish(msg);
  }
}