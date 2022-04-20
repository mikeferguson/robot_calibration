/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/CaptureConfig.h>

#include "robot_calibration/capture/capture_manager.h"
#include "robot_calibration/capture/poses.h"
#include "robot_calibration/calibration/export.h"

#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/load_bag.h>

/** \mainpage
 * \section parameters Parameters of the Optimization:
 *   - joint angle offsets
 *   - frame 6DOF corrections (currently head pan frame, and camera frame)
 *   - camera intrinsics (2d & 3d)
 *
 * \section residuals Residual Blocks:
 *   - difference of reprojection through the arm and camera
 *   - residual blocks that limit offsets from growing outrageously large
 *
 * \section modules Modules:
 *   - Capture:
 *     - move joints to a particular place
 *     - wait to settle
 *     - find target (led or checkerboard)
 *     - write sample to bag file: joint angles, position of targets in camera.
 *   - Calibrate:
 *     - load urdf, samples from bag file.
 *     - create arm and camera reprojection chains.
 *     - create residual blocks.
 *     - run calibration.
 *     - write results to URDF.
 */

/*
 * usage:
 *  calibrate --manual
 *  calibrate calibration_poses.bag
 *  calibrate --from-bag calibration_data.bag
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv,"robot_calibration");
  ros::NodeHandle nh("~");

  // Should we be stupidly verbose?
  bool verbose;
  nh.param<bool>("verbose", verbose, false);

  // The calibration data
  std_msgs::String description_msg;
  std::vector<robot_calibration_msgs::CalibrationData> data;

  // What bag to use to load calibration poses out of (for capture)
  std::string pose_bag_name("calibration_poses.bag");
  if (argc > 1)
    pose_bag_name = argv[1];

  if (pose_bag_name.compare("--from-bag") != 0)
  {
    // No name provided for a calibration bag file, must do capture
    robot_calibration::CaptureManager capture_manager;
    capture_manager.init(nh);

    // Save URDF for calibration/export step
    description_msg.data = capture_manager.getUrdf();

    // Load a set of calibration poses
    std::vector<robot_calibration_msgs::CaptureConfig> poses;
    if (pose_bag_name.compare("--manual") != 0)
    {
      if (!robot_calibration::getPosesFromBag(pose_bag_name, poses))
      {
        // Error will be printed in function
        return -1;
      }
    }
    else
    {
      ROS_INFO("Using manual calibration mode...");
    }

    // For each pose in the capture sequence.
    for (unsigned pose_idx = 0;
         (pose_idx < poses.size() || poses.size() == 0) && ros::ok();
         ++pose_idx)
    {
      if (poses.size() == 0)
      {
        // Manual calibration, wait for keypress
        ROS_INFO("Press [Enter] to capture a sample... (or type 'done' and [Enter] to finish capture)");
        std::string throwaway;
        std::getline(std::cin, throwaway);
        if (throwaway.compare("done") == 0)
          break;
        if (throwaway.compare("exit") == 0)
          return 0;
        if (!ros::ok())
          break;
      }
      else
      {
        // Move head/arm to pose
        if (!capture_manager.moveToState(poses[pose_idx].joint_states))
        {
          ROS_WARN("Unable to move to desired state for sample %u.", pose_idx);
          continue;
        }
      }

      // Make sure sensor data is up to date after settling
      ros::Duration(0.1).sleep();

      // Get pose of the features
      robot_calibration_msgs::CalibrationData msg;
      if (!capture_manager.captureFeatures(poses[pose_idx].features, msg))
      {
        ROS_WARN("Failed to capture sample %u.", pose_idx);
        continue;
      }

      ROS_INFO("Captured pose %u", pose_idx);

      // Add to samples
      data.push_back(msg);
    }

    ROS_INFO("Done capturing samples");
  }
  else
  {
    // Load calibration data from bagfile
    std::string data_bag_name("/tmp/calibration_data.bag");
    if (argc > 2)
      data_bag_name = argv[2];
    ROS_INFO_STREAM("Loading calibration data from " << data_bag_name);

    if (!robot_calibration::load_bag(data_bag_name, description_msg, data))
    {
      // Error will have been printed in function
      return -1;
    }
  }

  // Create instance of optimizer
  robot_calibration::OptimizationParams params;
  robot_calibration::Optimizer opt(description_msg.data);

  // Load calibration steps (if any)
  XmlRpc::XmlRpcValue cal_steps;
  if (nh.getParam("cal_steps", cal_steps))
  {
    // Should be a struct (mapping name -> config)
    if (cal_steps.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_FATAL("Parameter 'cal_steps' should be a struct.");
      return false;
    }

    XmlRpc::XmlRpcValue::iterator it;
    size_t step;
    size_t max_step = (cal_steps.size()>0)?cal_steps.size():1;
    std::vector<std::string> prev_frame_names;
    std::string prev_params_yaml;
    for (step = 0, it = cal_steps.begin(); step < max_step; step++, it++)
    {
      std::string name = static_cast<std::string>(it->first);
      ros::NodeHandle cal_steps_handle(nh, "cal_steps/"+name);
      params.LoadFromROS(cal_steps_handle);
      opt.optimize(params, data, verbose);
      if (verbose)
      {
        std::cout << "Parameter Offsets:" << std::endl;
        std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
      }
    }
  }
  else
  {
    // Single step calibration
    params.LoadFromROS(nh);
    opt.optimize(params, data, verbose);
    if (verbose)
    {
      std::cout << "Parameter Offsets:" << std::endl;
      std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
    }
  }

  // Write outputs
  robot_calibration::exportResults(opt, description_msg.data, data);

  ROS_INFO("Done calibrating");

  return 0;
}
