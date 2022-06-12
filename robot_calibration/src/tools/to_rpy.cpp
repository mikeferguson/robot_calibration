/*
 * Copyright (C) 2018 Michael Ferguson
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

#include <iostream>
#include <stdio.h>
#include <robot_calibration/models/chain.h>

/*
 * usage:
 *  to_rpy a b c
 */
int main(int argc, char** argv)
{
  if (argc < 4)
  {
  	std::cerr << "to_rpy: Converts axis-magnitude to RPY notation" << std::endl;
  	std::cerr << std::endl;
  	std::cerr << "usage: to_rpy a b c" << std::endl;
  	std::cerr << std::endl;
  	return -1;
  }
  double x = atof(argv[1]);
  double y = atof(argv[2]);
  double z = atof(argv[3]);

  KDL::Rotation r;
  r = robot_calibration::rotation_from_axis_magnitude(x, y, z);

  r.GetRPY(x, y, z);
  std::cout << x << ", " << y << ", " << z << std::endl;
  return 0;
}
