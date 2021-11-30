#ifndef ROBOT_CALIBRATION_CAPTURE_FEATURES_H
#define ROBOT_CALIBRATION_CAPTURE_FEATURES_H


/**
* @brief      Verifies the required parameters have been set and, if not, set default values.
*
* @param[in]  node  node handle reference
*
* @return     none
*/
void check_parameters(ros::NodeHandle& node);


/**
* @brief      Executes the steps required to automatically capture the feature from a bag of predetermined pose.
*             This includes commanding the robot to the specified pose then capturing the feature. 
*
* @param[in]  node  node handle reference
* @param[in]  data  vector reference for where the CalibrationData messages are to be stored.
*
* @return     bool indicating whether the run was successful or not     
*/
bool run_automatic_capture(ros::NodeHandle& node, std::vector<robot_calibration_msgs::CalibrationData>& data);

/**
* @brief      Executes the steps required to manually capture the feature from a bag of predetermined pose.
*             This includes waiting for the user to move the robot then capturing the feature on their command. 
*
* @param[in]  node  node handle reference
* @param[in]  data  vector reference for where the CalibrationData messages are to be stored.
*
* @return     bool indicating whether the run was successful or not     
*/
bool run_manual_capture(ros::NodeHandle& nh, std::vector<robot_calibration_msgs::CalibrationData>& data);


// TODO:
bool save_data(std::string file_name); // TODO: bool is not the data type 
bool start_bag(std::string msg_name);
bool stop_bag(std::string msg_name);
std::vector<sensor_msgs::JointState> load_calibration_poses();
bool move_to_position();
bool wait_for_manual_move();
bool capture_pose();
bool capture_calibration_data();

#endif  // ROBOT_CALIBRATION_CAPTURE_FEATURES_H