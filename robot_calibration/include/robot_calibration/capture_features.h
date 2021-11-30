#ifndef ROBOT_CALIBRATION_CAPTURE_FEATURES_H
#define ROBOT_CALIBRATION_CAPTURE_FEATURES_H


/**
* @brief      Verifies the required parameters have been set and, if not, set default values.
*
* @param[in]  node  node handle reference
*
* @return     bool indicating whether the parameters are all set
*/
bool check_parameters(ros::NodeHandle& node);


/**
* @brief      Creates a string for the absoulte path for a file in the home directory.
*
* @param[in]  local_dir     name of the local directory.
*
* @return     string representing the absolute path to the file
*/
std::string get_absolute_directory(std::string local_dir);


/**
* @brief      Executes the steps required to automatically capture the feature from a bag of predetermined pose.
*             This includes commanding the robot to the specified pose then capturing the feature. 
*
* @param[in]  node  node handle reference
* @param[in]  data  vector reference for where the CalibrationData messages are to be stored.
*
* @return     bool indicating whether the run was successful 
*/
bool run_automatic_capture(ros::NodeHandle& node, std::vector<robot_calibration_msgs::CalibrationData>& data);


/**
* @brief      Executes the steps required to manually capture the feature from a bag of predetermined pose.
*             This includes waiting for the user to move the robot then capturing the feature on their command. 
*
* @param[in]  node  node handle reference
* @param[in]  data  vector reference for where the CalibrationData messages are to be stored.
*
* @return     bool indicating whether the run was successful   
*/
bool run_manual_capture(ros::NodeHandle& nh, std::vector<robot_calibration_msgs::CalibrationData>& data);


/**
* @brief      Opens the file and creates a bag object.
*
* @param[in]  filename  absolute path to the file.
* @param[in]  bag       the bag object reference
*
* @return     bool indicating whether openning the file was successful
*/
bool open_bag(std::string filename, rosbag::Bag& bag);


/**
* @brief      Creates the bag object.
*
* @param[in]  filename  absolute path to the file.
* @param[in]  bag       the bag object reference
*
* @return     bool indicating whether creating the file was successful
*/
bool create_bag(std::string filename, rosbag::Bag& bag);


/**
* @brief      Creates an ordered vector of JointState poses/
*
* @param[in]  filename  absolute path to the bag file.
* @param[in]  pose      vector to store the individual poses in.
*
* @return     bool indicating whether creating the vector was successful
*/
bool load_calibration_poses(std::string filename, std::vector<robot_calibration_msgs::CaptureConfig>& poses);


bool move_to_position();

/**
* @brief      Creates a single CalibrationData message for a single feature at a single pose.
*
* @param[in]  chain_manager     manages the kinematic chains.
* @param[in]  finders           map of the available feature finders.
* @param[in]  feature           name of the feature to search for.
* @param[in]  msg               location to store new data message.
*
* @return     CalibrationData message for the capture    
*/
bool capture_calibration_data(robot_calibration::ChainManager& chain_manager, 
                                                                 robot_calibration::FeatureFinderMap& finders, 
                                                                 std::string& feature,
                                                                 robot_calibration_msgs::CalibrationData& msg);

#endif  // ROBOT_CALIBRATION_CAPTURE_FEATURES_H