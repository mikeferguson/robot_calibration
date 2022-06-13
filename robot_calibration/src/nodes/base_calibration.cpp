/*
 * Copyright (C) 2022 Michael Ferguson
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

  robot_calibration::BaseCalibration b;
  b.clearMessages();

  bool verbose = b.declare_parameter<bool>("verbose", false);

  // Rotate at several different speeds
  b.spin(0.5, 1, verbose);
  b.spin(1.5, 1, verbose);
  b.spin(3.0, 2, verbose);
  b.spin(-0.5, 1, verbose);
  b.spin(-1.5, 1, verbose);
  b.spin(-3.0, 2, verbose);

  // TODO: drive towards wall, to calibrate rollout

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
    std::string cal = b.printCalibrationData();
    file << cal;
    file.close();
    std::cout << cal;
  }

  return 0;
}
