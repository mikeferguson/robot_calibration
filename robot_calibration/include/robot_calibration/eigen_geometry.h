/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2015-2017 Fetch Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_EIGEN_GEOMETRY_H
#define ROBOT_CALIBRATION_EIGEN_GEOMETRY_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>

namespace robot_calibration
{

/**
 * @brief Get an Eigen::MatrixXd from a vector of PointStamped
 */
inline Eigen::MatrixXd getMatrix(const std::vector<geometry_msgs::PointStamped>& points)
{
  Eigen::MatrixXd matrix(3, points.size());
  for (size_t i = 0; i < points.size(); ++i)
  {
    matrix(0, i) = points[i].point.x;
    matrix(1, i) = points[i].point.y;
    matrix(2, i) = points[i].point.z;
  }
  return matrix;
}

/**
 * @brief Get the centroid of a point cloud.
 */
inline Eigen::Vector3d getCentroid(Eigen::MatrixXd points)
{
  return Eigen::Vector3d(points.row(0).mean(), points.row(1).mean(), points.row(2).mean()); 
}

/**
 * @brief Find the plane parameters for a point cloud
 * @param points Point cloud to determine plane parameters
 * @param normal The calculated normal, returned by reference
 * @param d The calculated d value in the plane equation
 *
 * The equation of the plane will be ax + by + cz + d = 0, where
 * a, b, and c are the normal.
 */
inline bool getPlane(Eigen::MatrixXd points, Eigen::Vector3d& normal, double& d)
{
  // Find centroid
  Eigen::Vector3d centroid = getCentroid(points);

  // Center the cloud
  points.row(0).array() -= centroid(0);
  points.row(1).array() -= centroid(1);
  points.row(2).array() -= centroid(2);

  // Find the plane
  auto svd = points.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  normal = svd.matrixU().rightCols<1>();

  // Get the rest of plane equation
  d = -(normal(0) * centroid(0) + normal(1) * centroid(1) + normal(2) * centroid(2));

  if (d < 0)
  {
    // Invert the normal vector so that d is always positive
    d *= -1;
    normal *= -1;
  }

  return true;
}

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_EIGEN_GEOMETRY_H
