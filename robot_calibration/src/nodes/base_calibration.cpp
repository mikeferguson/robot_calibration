/*
 * Copyright (C) 2022-2024 Michael Ferguson
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2014 Unbounded Robotics Inc.
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

#include <cmath>
#include <fstream>
#include "robot_calibration/optimization/base_calibration.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<robot_calibration::BaseCalibration> b =
    std::make_shared<robot_calibration::BaseCalibration>();
  b->clearMessages();

  bool verbose = b->declare_parameter<bool>("verbose", false);

  // Load parameters
  auto calibration_steps = b->declare_parameter<std::vector<std::string>>(
    "calibration_steps", std::vector<std::string>());

  if (calibration_steps.empty())
  {
    RCLCPP_WARN(b->get_logger(), "No calibration_steps defined, using defaults");
    // Rotate at several different speeds
    b->spin(0.5, 1, verbose);
    b->spin(1.5, 1, verbose);
    b->spin(3.0, 2, verbose);
    b->spin(-0.5, 1, verbose);
    b->spin(-1.5, 1, verbose);
    b->spin(-3.0, 2, verbose);
  }
  else
  {
    for (auto step : calibration_steps)
    {
      std::string step_type = b->declare_parameter<std::string>(step + ".type", "");
      if (step_type == "spin")
      {
        double velocity = b->declare_parameter<double>(step + ".velocity", 1.0);
        int rotations = b->declare_parameter<int>(step + ".rotations", 1);
        b->spin(velocity, rotations, verbose);
      }
      else if (step_type == "rollout")
      {
        double velocity = b->declare_parameter<double>(step + ".velocity", 1.0);
        double distance = b->declare_parameter<double>(step + ".distance", 0.0);
        b->rollout(velocity, distance, verbose);
      }
      else
      {
        RCLCPP_ERROR(b->get_logger(), "Unrecognized step type: %s", step_type.c_str());
      }
    }
  }

  // Output yaml file
  {
    // Generate datecode
    char datecode[80];
    {
      std::time_t t = std::time(NULL);
      std::strftime(datecode, 80, "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
    }
    std::stringstream yaml_name;
    yaml_name << "/tmp/base_calibration_" << datecode << ".yaml";
    std::ofstream file;
    file.open(yaml_name.str().c_str());
    std::string cal = b->printCalibrationData();
    file << cal;
    file.close();
    std::cout << cal;
  }

  return 0;
}
