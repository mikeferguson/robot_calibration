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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>
#include <robot_calibration_msgs/msg/capture_config.hpp>

#include <robot_calibration/optimization/ceres_optimizer.hpp>
#include <robot_calibration/optimization/export.hpp>
#include <robot_calibration/util/calibration_data.hpp>
#include <robot_calibration/util/capture_manager.hpp>
#include <robot_calibration/util/poses_from_bag.hpp>

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
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_calibration");
  rclcpp::Logger logger = node->get_logger();

  // Should we be stupidly verbose?
  bool verbose = node->declare_parameter<bool>("verbose", false);

  // The calibration data
  std_msgs::msg::String description_msg;
  std::vector<robot_calibration_msgs::msg::CalibrationData> data;

  // What bag to use to load calibration poses out of (for capture)
  std::string pose_bag_name("calibration_poses.bag");
  if (argc > 1)
    pose_bag_name = argv[1];

  if (pose_bag_name.compare("--from-bag") != 0)
  {
    // No name provided for a calibration bag file, must do capture
    robot_calibration::CaptureManager capture_manager;
    capture_manager.init(node);

    // Save URDF for calibration/export step
    description_msg.data = capture_manager.getUrdf();

    // Load a set of calibration poses
    std::vector<robot_calibration_msgs::msg::CaptureConfig> poses;
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
      RCLCPP_INFO(logger, "Using manual calibration mode...");
    }

    // For each pose in the capture sequence.
    for (unsigned pose_idx = 0;
         (pose_idx < poses.size() || poses.empty()) && rclcpp::ok();
         ++pose_idx)
    {
      robot_calibration_msgs::msg::CalibrationData msg;
      if (poses.empty())
      {
        // Manual calibration, wait for keypress
        RCLCPP_INFO(logger, "Press [Enter] to capture a sample... (or type 'done' and [Enter] to finish capture)");
        std::string throwaway;
        std::getline(std::cin, throwaway);
        if (throwaway.compare("done") == 0)
          break;
        if (throwaway.compare("exit") == 0)
          return 0;
        if (!rclcpp::ok())
          break;

        // Empty vector causes us to capture all features
        std::vector<std::string> features;
        if (!capture_manager.captureFeatures(features, msg))
        {
          RCLCPP_WARN(logger, "Failed to capture sample %u.", pose_idx);
          continue;
        }
      }
      else
      {
        // Move head/arm to pose
        if (!capture_manager.moveToState(poses[pose_idx].joint_states))
        {
          RCLCPP_WARN(logger, "Unable to move to desired state for sample %u.", pose_idx);
          continue;
        }

        // Make sure sensor data is up to date after settling
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        // Get pose of the features
        if (!capture_manager.captureFeatures(poses[pose_idx].features, msg))
        {
          RCLCPP_WARN(logger, "Failed to capture sample %u.", pose_idx);
          continue;
        }
      }

      RCLCPP_INFO(logger, "Captured pose %u", pose_idx);

      // Add to samples
      data.push_back(msg);
    }

    RCLCPP_INFO(logger, "Done capturing samples");
  }
  else
  {
    // Load calibration data from bagfile
    std::string data_bag_name("/tmp/calibration_data.bag");
    if (argc > 2)
      data_bag_name = argv[2];
    RCLCPP_INFO(logger, "Loading calibration data from %s", data_bag_name.c_str());

    if (!robot_calibration::load_bag(data_bag_name, description_msg, data))
    {
      // Error will have been printed in function
      return -1;
    }
  }

  // Create instance of optimizer
  robot_calibration::OptimizationParams params;
  robot_calibration::Optimizer opt(description_msg.data);

  // Load calibration steps
  std::vector<std::string> calibration_steps =
    node->declare_parameter<std::vector<std::string>>("calibration_steps", std::vector<std::string>());
  if (calibration_steps.empty())
  {
    RCLCPP_FATAL(logger, "Parameter calibration_steps is not defined");
    return -1;
  }

  // Run calibration steps
  for (auto step : calibration_steps)
  {
    params.LoadFromROS(node, step);
    opt.optimize(params, data, logger, verbose);
    if (verbose)
    {
      std::cout << "Parameter Offsets:" << std::endl;
      std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
    }
  }

  // Write outputs
  robot_calibration::exportResults(opt, description_msg.data, data);

  RCLCPP_INFO(logger, "Done calibrating");
  rclcpp::shutdown();

  return 0;
}
