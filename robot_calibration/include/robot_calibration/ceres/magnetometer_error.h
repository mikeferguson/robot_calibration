/*
 * Copyright (C) 2020 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_CERES_MAGNETOMETER_ERROR_H
#define ROBOT_CALIBRATION_CERES_MAGNETOMETER_ERROR_H

#include <ceres/ceres.h>

/**
 * @brief Cost functor for magnetometer sample
 *        when doing only hard iron offsets.
 */
class HardIronOffsetError
{
public:
  HardIronOffsetError(double x, double y, double z)
  {
    x_ = x;
    y_ = y;
    z_ = z;
  }

  template <typename T>
  bool operator()(const T* const params, T* residuals) const
  {
    // Parameters are:
    //  0 = local magnetic field strength
    //  1 = x offset
    //  2 = y offset
    //  3 = z offset
    residuals[0] = (x_ - params[1]) * (x_ - params[1]) +
                   (y_ - params[2]) * (y_ - params[2]) +
                   (z_ - params[3]) * (z_ - params[3]) -
                   (params[0] * params[0]);

    return true;
  }

  static ceres::CostFunction* Create(double x, double y, double z)
  {
    ceres::CostFunction* func
      = new ceres::AutoDiffCostFunction<HardIronOffsetError, 1, 4>(new HardIronOffsetError(x, y, z));
    return func;
  }

private:
  // The actual sampled data
  double x_, y_, z_;
};

#endif  // ROBOT_CALIBRATION_CERES_MAGNETOMETER_ERROR_H
