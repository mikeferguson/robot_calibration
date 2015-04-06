/*
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

#ifndef ROBOT_CALIBRATION_CERES_DATA_FUNCTIONS_H
#define ROBOT_CALIBRATION_CERES_DATA_FUNCTIONS_H

#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

/** \brief Compute the average point based on a vector of points. */
geometry_msgs::PointStamped
computeAverage(std::vector<geometry_msgs::PointStamped> points)
{
  geometry_msgs::PointStamped p;
  p.header = points[0].header;

  for (size_t i = 0; i < points.size(); ++i)
  {
    p.point.x += points[i].point.x;
    p.point.y += points[i].point.y;
    p.point.z += points[i].point.z;
  }

  p.point.x = p.point.x / points.size();
  p.point.y = p.point.y / points.size();
  p.point.z = p.point.z / points.size();

  return p;
}

/** \brief Compute the average value based on a vector of doubles. */
double computeAverage(std::vector<double> values)
{
  double x = 0.0;
  for (size_t i = 0; i < values.size(); ++i)
    x += values[i];
  return (x/values.size()); 
}

/**
 *  \brief Compute the difference in between two points.
 *         Note that points must be in the same frame.
 */
geometry_msgs::PointStamped
computeDifference(geometry_msgs::PointStamped& p1,
                  geometry_msgs::PointStamped& p2)
{
  geometry_msgs::PointStamped p;
  p.header = p1.header;

  p.point.x = fabs(p1.point.x - p2.point.x);
  p.point.y = fabs(p1.point.y - p2.point.y);
  p.point.z = fabs(p1.point.z - p2.point.z);

  return p;
}

/**
 *  \brief Compute the distance in Euclidean space between two points.
 *         Note that points must be in the same frame.
 */
double getDistance(geometry_msgs::PointStamped& p1,
                   geometry_msgs::PointStamped& p2)
{
  double x = p1.point.x - p2.point.x;
  double y = p1.point.y - p2.point.y;
  double z = p1.point.z - p2.point.z;
  return sqrt(x*x + y*y + z*z);
}

/** \brief Returns the distance between each point in a given sample. */
std::vector<double> getErrors(ChainModel * chain1,
                              ChainModel * chain2,
                              CalibrationOffsetParser * offsets,
                              robot_calibration_msgs::CalibrationData& data)
{
  std::vector<double> error;
  std::vector<geometry_msgs::PointStamped> proj1 = chain1->project(data, *offsets);
  std::vector<geometry_msgs::PointStamped> proj2 = chain2->project(data, *offsets);

  for (size_t i = 0; i < proj1.size(); ++i)
  {
    error.push_back(getDistance(proj1[i], proj2[i]));
  }

  return error;
}

/**
 *  \brief Returns a vector of points representing the differences between
 *         two projections using different chains but the same calibration
 *         offsets.
 */
std::vector<geometry_msgs::PointStamped>
getErrorPoints(ChainModel * chain1,
               ChainModel * chain2,
               CalibrationOffsetParser * offsets,
               robot_calibration_msgs::CalibrationData& data)
{
  std::vector<geometry_msgs::PointStamped> error;
  std::vector<geometry_msgs::PointStamped> proj1 = chain1->project(data, *offsets);
  std::vector<geometry_msgs::PointStamped> proj2 = chain2->project(data, *offsets);

  for (size_t i = 0; i < proj1.size(); ++i)
  {
    error.push_back(computeDifference(proj1[i], proj2[i]));
  }

  return error;
}

void printSimpleDistanceError(ChainModel * chain1,
                              ChainModel * chain2,
                              CalibrationOffsetParser * before,
                              CalibrationOffsetParser * after,
                              robot_calibration_msgs::CalibrationData& data)
{
  std::cout << "  Distance Error Before: " << computeAverage(getErrors(chain1, chain2, before, data)) <<
                 ", After: " << computeAverage(getErrors(chain1, chain2, after, data)) << std::endl;
}

void printComparePointsInternal(ChainModel * chain1,
                                ChainModel * chain2,
                                CalibrationOffsetParser * offsets,
                                robot_calibration_msgs::CalibrationData& data)
{
  std::vector<geometry_msgs::PointStamped> proj1 = chain1->project(data, *offsets);
  std::vector<geometry_msgs::PointStamped> proj2 = chain2->project(data, *offsets);

  std::cout << "  x:";
  for (size_t x = 0; x < proj1.size(); ++x)
    std::cout << "  " << std::setw(10) << std::fixed << proj1[x].point.x;
  std::cout << "  |  ";
  for (size_t x = 0; x < proj2.size(); ++x)
    std::cout << "  " << std::setw(10) << std::fixed << proj2[x].point.x;
  std::cout << std::endl;

  std::cout << "  y:";
  for (size_t y = 0; y < proj1.size(); ++y)
    std::cout << "  " << std::setw(10) << std::fixed << proj1[y].point.y;
  std::cout << "  |  ";
  for (size_t y = 0; y < proj2.size(); ++y)
    std::cout << "  " << std::setw(10) << std::fixed << proj2[y].point.y;
  std::cout << std::endl;

  std::cout << "  z:";
  for (size_t z = 0; z < proj1.size(); ++z)
    std::cout << "  " << std::setw(10) << std::fixed << proj1[z].point.z;
  std::cout << "  |  ";
  for (size_t z = 0; z < proj2.size(); ++z)
    std::cout << "  " << std::setw(10) << std::fixed << proj2[z].point.z;
  std::cout << std::endl;
}

void printComparePoints(ChainModel * chain1,
                        ChainModel * chain2,
                        CalibrationOffsetParser * before,
                        CalibrationOffsetParser * after,
                        robot_calibration_msgs::CalibrationData& data)
{
  std::cout << "  Points Before:" << std::endl;
  printComparePointsInternal(chain1, chain2, before, data);
  std::cout << std::endl;
  std::cout << "  Points After:" << std::endl;
  printComparePointsInternal(chain1, chain2, after, data);
  std::cout << std::endl;

  // error by dimension
  geometry_msgs::PointStamped err_before = computeAverage(getErrorPoints(chain1, chain2, before, data));
  geometry_msgs::PointStamped err_after = computeAverage(getErrorPoints(chain1, chain2, after, data));

  std::cout << "  Error:     Before       After" << std::endl;
  std::cout << "  x:     " << std::setw(10) << std::fixed << err_before.point.x <<
               "  " << std::setw(10) << std::fixed << err_after.point.x << std::endl;
  std::cout << "  y:     " << std::setw(10) << std::fixed << err_before.point.y <<
               "  " << std::setw(10) << std::fixed << err_after.point.y << std::endl;
  std::cout << "  z:     " << std::setw(10) << std::fixed << err_before.point.z <<
               "  " << std::setw(10) << std::fixed << err_after.point.z << std::endl;
  std::cout << std::endl; 
}

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_DATA_FUNCTIONS_H
