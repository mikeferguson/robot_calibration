/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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

#include <robot_calibration/ceres/optimizer.h>

#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/ceres/calibration_data_helpers.h>
#include <robot_calibration/ceres/chain3d_to_chain3d_error.h>
#include <robot_calibration/ceres/chain3d_to_mesh_error.h>
#include <robot_calibration/ceres/chain3d_to_plane_error.h>
#include <robot_calibration/ceres/plane_to_plane_error.h>
#include <robot_calibration/ceres/outrageous_error.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

namespace robot_calibration
{

Optimizer::Optimizer(const std::string& robot_description) :
  num_params_(0),
  num_residuals_(0)
{
  if (!model_.initString(robot_description))
    std::cerr << "Failed to parse URDF." << std::endl;

  // Maintain consistent offset parser so we hold onto offsets
  offsets_.reset(new CalibrationOffsetParser());

  // Create a mesh loader
  mesh_loader_.reset(new MeshLoader(model_));
}

Optimizer::~Optimizer()
{
}

int Optimizer::optimize(OptimizationParams& params,
                        std::vector<robot_calibration_msgs::CalibrationData> data,
                        bool progress_to_stdout)
{
  // Load KDL from URDF
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    return -1;
  }

  // Create models
  for (size_t i = 0; i < params.models.size(); ++i)
  {
    if (params.models[i].type == "chain")
    {
      ROS_INFO_STREAM("Creating chain '" << params.models[i].name << "' from " <<
                                            params.base_link << " to " <<
                                            params.models[i].params["frame"]);
      ChainModel* model = new ChainModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"]);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "camera3d")
    {
      ROS_INFO_STREAM("Creating camera3d '" << params.models[i].name << "' in frame " <<
                                               params.models[i].params["frame"]);
      std::string param_name = params.models[i].params["param_name"];
      if (param_name == "")
      {
        // Default to same name as sensor
        param_name = params.models[i].name;
      }
      Camera3dModel* model = new Camera3dModel(params.models[i].name, param_name, tree_, params.base_link, params.models[i].params["frame"]);
      models_[params.models[i].name] = model;
    }
    else
    {
      // ERROR unknown
    }
  }

  // Reset which parameters are free (offset values are retained)
  offsets_->reset();

  // Setup  parameters to calibrate
  for (size_t i = 0; i < params.free_params.size(); ++i)
  {
    offsets_->add(params.free_params[i]);
  }
  for (size_t i = 0; i < params.free_frames.size(); ++i)
  {
    offsets_->addFrame(params.free_frames[i].name,
                       params.free_frames[i].x,
                       params.free_frames[i].y,
                       params.free_frames[i].z,
                       params.free_frames[i].roll,
                       params.free_frames[i].pitch,
                       params.free_frames[i].yaw);
  }
  for (size_t i = 0; i < params.free_frames_initial_values.size(); ++i)
  {
    if (!offsets_->setFrame(params.free_frames_initial_values[i].name,
                            params.free_frames_initial_values[i].x,
                            params.free_frames_initial_values[i].y,
                            params.free_frames_initial_values[i].z,
                            params.free_frames_initial_values[i].roll,
                            params.free_frames_initial_values[i].pitch,
                            params.free_frames_initial_values[i].yaw))
    {
      ROS_ERROR_STREAM("Error setting initial value for " <<
                       params.free_frames_initial_values[i].name);
    }
  }

  // Allocate space
  double* free_params = new double[offsets_->size()];
  offsets_->initialize(free_params);

  // Houston, we have a problem...
  ceres::Problem* problem = new ceres::Problem();

  // For each sample of data:
  for (size_t i = 0; i < data.size(); ++i)
  {
    for (size_t j = 0; j < params.error_blocks.size(); ++j)
    {
      if (params.error_blocks[j].type == "chain3d_to_chain3d")
      {
        // This error block can process data generated by the LedFinder,
        // CheckboardFinder, or any other finder that can sample the pose
        // of one or more data points that are connected at a constant offset
        // from a link a kinematic chain (the "arm").
        std::string a_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);
        std::string b_name = static_cast<std::string>(params.error_blocks[j].params["model_b"]);

        // Do some basic error checking for bad params
        if (a_name == "" || b_name == "" || a_name == b_name)
        {
          ROS_ERROR("chain3d_to_chain3d improperly configured: model_a and model_b params must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], a_name) || !hasSensor(data[i], b_name))
          continue;

        // Create the block
        ceres::CostFunction * cost = Chain3dToChain3d::Create(models_[a_name],
                                                              models_[b_name],
                                                              offsets_.get(),
                                                              data[i]);

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);

          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  x: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 3); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 0)];
          std::cout << std::endl << "  y: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 3); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 1)];
          std::cout << std::endl << "  z: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 3); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 2)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL,  // squared loss
                                  free_params);
      }
      else if (params.error_blocks[j].type == "chain3d_to_plane")
      {
        // This error block can process data generated by the PlaneFinder
        std::string chain_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);

        // Do some basic error checking for bad params
        if (chain_name == "")
        {
          ROS_ERROR("chain3d_to_plane improperly configured: model_a param must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], chain_name))
          continue;

        // Create the block
        ceres::CostFunction * cost =
          Chain3dToPlane::Create(models_[chain_name],
                                 offsets_.get(),
                                 data[i],
                                 params.getParam(params.error_blocks[j], "a", 0.0),
                                 params.getParam(params.error_blocks[j], "b", 0.0),
                                 params.getParam(params.error_blocks[j], "c", 1.0),
                                 params.getParam(params.error_blocks[j], "d", 0.0),
                                 params.getParam(params.error_blocks[j], "scale", 1.0));

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);

          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  d: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals()); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(k)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL /* squared loss */,
                                  free_params);
      }
      else if (params.error_blocks[j].type == "chain3d_to_mesh")
      {
        // This error block can process data generated by the RobotFinder
        std::string chain_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);

        // Do some basic error checking for bad params
        if (chain_name == "")
        {
          ROS_ERROR("chain3d_to_mesh improperly configured: model_a param must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], chain_name))
          continue;

        // Get the mesh
        std::string link_name = params.getParam(params.error_blocks[j], "link_name", std::string(""));
        MeshPtr mesh = mesh_loader_->getCollisionMesh(link_name);
        if (!mesh)
        {
          ROS_ERROR("chain3d_to_mesh improperly configured: cannot load mesh for %s", link_name.c_str());
          return 0;
        }

        // Create the block
        ceres::CostFunction * cost =
          Chain3dToMesh::Create(models_[chain_name],
                                offsets_.get(),
                                data[i],
                                mesh);

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);

          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  d: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals()); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(k)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL /* squared loss */,
                                  free_params);


      }
      else if (params.error_blocks[j].type == "plane_to_plane")
      {
        // This error block can process data generated by the PlaneFinder,
        // CheckerboardFinder, or any other finder that returns a series of
        // planar points.
        std::string a_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);
        std::string b_name = static_cast<std::string>(params.error_blocks[j].params["model_b"]);

        // Do some basic error checking for bad params
        if (a_name == "" || b_name == "" || a_name == b_name)
        {
          ROS_ERROR("plane_to_plane improperly configured: model_a and model_a params must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], a_name) || !hasSensor(data[i], b_name))
          continue;

        // Create the block
        ceres::CostFunction * cost =
          PlaneToPlaneError::Create(models_[a_name],
                                    models_[b_name],
                                    offsets_.get(),
                                    data[i],
                                    params.getParam(params.error_blocks[j], "scale_normal", 1.0),
                                    params.getParam(params.error_blocks[j], "scale_offset", 1.0));

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);
          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  a: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[0];
          std::cout << std::endl << "  b: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[1];
          std::cout << std::endl << "  c: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[2];
          std::cout << std::endl << "  d: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[3];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL,  // squared loss
                                  free_params);
      }
      else if (params.error_blocks[j].type == "outrageous")
      {
        // Outrageous error block requires no particular sensors, add to every sample
        problem->AddResidualBlock(
          OutrageousError::Create(offsets_.get(),
                                  static_cast<std::string>(params.error_blocks[j].params["param"]),
                                  params.getParam(params.error_blocks[j], "joint_scale", 1.0),
                                  params.getParam(params.error_blocks[j], "position_scale", 1.0),
                                  params.getParam(params.error_blocks[j], "rotation_scale", 1.0)),
          NULL, // squared loss
          free_params);
      }
      else
      {
        ROS_ERROR_STREAM("Unknown error block: " << params.error_blocks[j].type);
        return 0;
      }
    }
  }

  // Setup the actual optimization
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1e-10;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = progress_to_stdout;
  //  options.use_nonmonotonic_steps = true;

  if (progress_to_stdout)
    std::cout << "\nSolver output:" << std::endl;
  summary_.reset(new ceres::Solver::Summary());
  ceres::Solve(options, problem, summary_.get());
  if (progress_to_stdout)
    std::cout << "\n" << summary_->BriefReport() << std::endl;

  // Save some status
  num_params_ = problem->NumParameters();
  num_residuals_ = problem->NumResiduals();

  // Note: the error blocks will be managed by scoped_ptr in cost functor
  //       which takes ownership, and so we do not need to delete them here

  // Done with our free params
  delete[] free_params;
  delete problem;

  return 0;
}

}  // namespace robot_calibration
