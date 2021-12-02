#ifndef ROBOT_CALIBRATION_OPTIMIZE_PARAMETERS_H
#define ROBOT_CALIBRATION_OPTIMIZE_PARAMETERS_H


/**
* @brief      Verifies the required parameters have been set and, if not, set default values.
*
* @param[in]  node  node handle reference
*
* @return     bool indicating whether the parameters are all set.
*/
bool check_parameters(ros::NodeHandle& node);


/**
* @brief      Creates a string for the absoulte path for a file in the home directory.
*
* @param[in]  local_dir     name of the local directory.
*
* @return     string representing the absolute path to the file.
*/
std::string get_absolute_directory(std::string local_dir);


/**
* @brief      Organizes loading data from the rosbag file.
*
* @param[in]  bag_filename      absolute path to the rosbag file.
* @param[in]  description_msg   holds the robot description string.
* @param[in]  data              holds the calibration data.
*
* @return     bool indicating whther data loading was successful.
*/
bool load_data(std::string& bag_filename, 
                           std_msgs::String& description_msg,
                           std::vector<robot_calibration_msgs::CalibrationData>& data);


/**
* @brief      Opens the rosbag at the given location.
*
* @param[in]  bag               rosbag object.
* @param[in]  bag_filename      absolute path to the rosbag file.
*
* @return     bool indicating whether the rosbag could be openned.
*/
bool open_rosbag(rosbag::Bag& bag, std::string& bag_filename);


/**
* @brief      Loads the robot description from the rosbag file.
*
* @param[in]  bag               rosbag object.
* @param[in]  description_msg   place to load the robot description from the rosbag.
*
* @return     bool indicating whether the data could be loaded.
*/
bool load_robot_description(rosbag::Bag& bag, std_msgs::String& description_msg);


/**
* @brief      Loads the calibration data from the rosbag file.
*
* @param[in]  bag               rosbag object.
* @param[in]  description_msg   place to load the robot description from the rosbag.
*
* @return     bool indicating whether the data could be loaded.
*/
bool load_calibration_data(rosbag::Bag& bag, std::vector<robot_calibration_msgs::CalibrationData>& data);

#endif  // ROBOT_CALIBRATION_OPTIMIZE_PARAMETERS_H