
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/optimize_parameters.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv,"robot_calibration");
  ros::NodeHandle nh("~");

  // Parameters
  std::string bag_filename;                                   // Absolute path to location to save the bagfile
  bool verbose;                                               // Level of data to output to the screen

  if(!optimize_parameters::check_parameters(nh))
    return -1;

  nh.getParam("bag_filename", bag_filename); 
  nh.getParam("verbose", verbose);

  // Load data
  ROS_INFO("Loading optimization data");
  std_msgs::String description_msg;
  std::vector<robot_calibration_msgs::CalibrationData> data;

  if(!optimize_parameters::load_data(bag_filename, description_msg, data))
    return -1;

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
