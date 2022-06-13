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

#ifndef ROBOT_CALIBRATION_COST_FUNCTIONS_CHAIN3D_TO_MESH_ERROR_HPP
#define ROBOT_CALIBRATION_COST_FUNCTIONS_CHAIN3D_TO_MESH_ERROR_HPP

#include <limits>
#include <string>
#include <math.h>
#include <ceres/ceres.h>
#include <robot_calibration/models/camera3d.hpp>
#include <robot_calibration/models/chain3d.hpp>
#include <robot_calibration/optimization/offset_parser.h>
#include <robot_calibration/util/calibration_data.hpp>
#include <robot_calibration/util/mesh_loader.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>

namespace robot_calibration
{

/**
 * \brief Get the squared distance line segment A-B for point C
 * \param a Point representing one end of the line segment
 * \param b Point representing the other end of the line segment
 * \param c Point to get distance to line segment
 *
 * Based on "Real Time Collision Detection", pg 130
 */
double distToLine(Eigen::Vector3d& a, Eigen::Vector3d& b, Eigen::Vector3d c)
{
  Eigen::Vector3d ab = b - a;
  Eigen::Vector3d ac = c - a;
  Eigen::Vector3d bc = c - b;

  double e = ac.dot(ab);
  if (e <= 0.0)
  {
    // Point A is closest to C
    return ac.dot(ac);
  }
  double f = ab.dot(ab);
  if (e >= f)
  {
    // Point B is closest to C
    return bc.dot(bc);
  }
  // C actually projects between
  return ac.dot(ac) - e * e / f;
}

/**
 *  \brief Error block for computing the fit between a set of projected
 *         points and a mesh (usually part of the robot body). Typically used
 *         to align sensor with the robot footprint.
 */
struct Chain3dToMesh
{
  /**
   *  \brief This function is not used direcly, instead use the Create() function.
   *  \param chain_model The model for the chain, used for reprojection.
   *  \param offsets Easy access to the free parameters.
   *  \param data The calibration data collected.
   *  \param mesh_path Path to the mesh file to test against
   */
  Chain3dToMesh(Chain3dModel* chain_model,
                CalibrationOffsetParser* offsets,
                robot_calibration_msgs::msg::CalibrationData& data,
                MeshPtr& mesh)
  {
    chain_model_ = chain_model;
    offsets_ = offsets;
    data_ = data;
    mesh_ = mesh;
  }

  virtual ~Chain3dToMesh() {}

  bool operator()(double const * const * free_params,
                  double* residuals) const
  {
    // Update calibration offsets based on free params
    offsets_->update(free_params[0]);

    // Project the camera observations
    std::vector<geometry_msgs::msg::PointStamped> chain_pts =
        chain_model_->project(data_, *offsets_);

    // Compute residuals
    for (size_t pt = 0; pt < chain_pts.size() ; ++pt)
    {
      Eigen::Vector3d p(chain_pts[pt].point.x, chain_pts[pt].point.y, chain_pts[pt].point.z);

      // Find shortest distance to any line segment forming a triangle
      double dist = std::numeric_limits<double>::max();
      for (size_t t = 0; t < mesh_->triangle_count; ++t)
      {
        // Get the index of each vertex of the triangle
        int A_idx = mesh_->triangles[(3 * t) + 0];
        int B_idx = mesh_->triangles[(3 * t) + 1];
        int C_idx = mesh_->triangles[(3 * t) + 2];
        // Get the vertices
        Eigen::Vector3d A(mesh_->vertices[(3 * A_idx) + 0], mesh_->vertices[(3 * A_idx) + 1], mesh_->vertices[(3 * A_idx) + 2]);
        Eigen::Vector3d B(mesh_->vertices[(3 * B_idx) + 0], mesh_->vertices[(3 * B_idx) + 1], mesh_->vertices[(3 * B_idx) + 2]);
        Eigen::Vector3d C(mesh_->vertices[(3 * C_idx) + 0], mesh_->vertices[(3 * C_idx) + 1], mesh_->vertices[(3 * C_idx) + 2]);
        // Compare each line segment
        double d = distToLine(A, B, p);
        d = std::min(d, distToLine(B, C, p));
        d = std::min(d, distToLine(C, A, p));
        dist = std::min(d, dist);
      }
      residuals[pt] = std::sqrt(dist);
    }
    return true;
  }

  /**
   *  \brief Helper factory function to create a new error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  static ceres::CostFunction* Create(Chain3dModel* a_model,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::msg::CalibrationData& data,
                                     MeshPtr mesh)
  {
    int index = getSensorIndex(data, a_model->getName());
    if (index == -1)
    {
      // In theory, we should never get here, because the optimizer does a check
      std::cerr << "Sensor name doesn't match any of the existing finders" << std::endl;
      return 0;
    }

    ceres::DynamicNumericDiffCostFunction<Chain3dToMesh> * func;
    func = new ceres::DynamicNumericDiffCostFunction<Chain3dToMesh>(
                    new Chain3dToMesh(a_model, offsets, data, mesh));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(data.observations[index].features.size());

    return static_cast<ceres::CostFunction*>(func);
  }

  Chain3dModel * chain_model_;
  CalibrationOffsetParser * offsets_;
  robot_calibration_msgs::msg::CalibrationData data_;
  MeshPtr mesh_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_COST_FUNCTIONS_CHAIN3D_TO_MESH_ERROR_HPP
