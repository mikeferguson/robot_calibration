#ifndef ROBOT_CALIBRATION_CAPTURE_FEATURES_H
#define ROBOT_CALIBRATION_CAPTURE_FEATURES_H


/**
* @brief      Verifies the required parameters have been set and, if not, set default values.
*
* @param[in]  node  node handle reference
*
* @return     bool indicating whether the parameters are all set
*/
bool check_parameters(const ros::NodeHandle& node);


/**
* @brief      Creates a string for the absoulte path for a file in the home directory.
*
* @param[in]  local_dir     name of the local directory.
*
* @return     string representing the absolute path to the file
*/
std::string get_absolute_directory(const std::string& local_dir);


/**
* @brief      Creates a map of the available feature finders
*
* @param[in]  nh        node handle reference.
*
* @return     bool indicating whether the feature finders were loaded 
*/
std::optional<robot_calibration::FeatureFinderMap> get_feature_finders(ros::NodeHandle& nh);


// TODO: describe why this is needed.
/**
* @brief      Gets the robot description and republishes it
*
* @param[in]  nh        node handle reference.
*
* @return     the published robot descripition message
*/
std_msgs::String republish_robot_description(ros::NodeHandle& nh);


/**
* @brief      Executes the steps required to automatically capture the feature from a bag of predetermined pose.
*             This includes commanding the robot to the specified pose then capturing the feature. 
*
* @param[in]  nh                node handle reference.
* @param[in]  feature           feature to use for calibration
* @param[in]  bag               rosbag object for storing calibration data
*
* @return     bool indicating whether the run was successful 
*/
bool run_automatic_capture(ros::NodeHandle& nh,
                           const std::string& feature,
                           rosbag::Bag& bag);


/**
* @brief      Executes the steps required to manually capture the feature from a bag of predetermined pose.
*             This includes waiting for the user to move the robot then capturing the feature on their command. 
*
* @param[in]  nh                node handle reference.
* @param[in]  feature           feature to use for calibration
* @param[in]  bag               rosbag object for storing calibration data
*
* @return     bool indicating whether the run was successful   
*/
bool run_manual_capture(ros::NodeHandle& nh,
                        const std::string& feature,
                        rosbag::Bag& bag);


/**
* @brief      Creates an ordered vector of JointState poses.
*
* @param[in]  filename  absolute path to the bag file.
*
* @return     vector of poses to be used for calibration.
*/
std::vector<robot_calibration_msgs::CaptureConfig> load_calibration_poses(const std::string& filename);


/**
* @brief      Opens the file and creates a bag object.
*
* @param[in]  filename  absolute path to the file.
*
* @return     the rosbag object.
*/
rosbag::Bag open_bag(const std::string& filename);


/**
* @brief      Creates the bag object.
*
* @param[in]  filename  absolute path to the file.
* @param[in]  bag       the bag object reference
*
* @return     bool indicating whether creating the file was successful
*/
bool create_bag(std::string filename, rosbag::Bag& bag);


bool move_to_position();

/**
* @brief      Creates a single CalibrationData message for a single feature at a single pose.
*
* @param[in]  chain_manager     manages the kinematic chains.
* @param[in]  finders           map of the available feature finders.
* @param[in]  feature           name of the feature to search for.
*
* @return     CalibrationData message for the capture    
*/
std::optional<robot_calibration_msgs::CalibrationData> capture_calibration_data(robot_calibration::ChainManager& chain_manager, 
                                                                                robot_calibration::FeatureFinderMap& finders, 
                                                                                const std::string& feature);

#endif  // ROBOT_CALIBRATION_CAPTURE_FEATURES_H