/*
 * Copyright (C) 2018-2023 Michael Ferguson
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

#include <robot_calibration/optimization/ceres_optimizer.hpp>

#include <memory>
#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>

#include <robot_calibration/optimization/offsets.hpp>
#include <robot_calibration/util/calibration_data.hpp>
#include <robot_calibration/cost_functions/chain3d_to_camera2d_error.hpp>
#include <robot_calibration/cost_functions/chain3d_to_chain3d_error.hpp>
#include <robot_calibration/cost_functions/chain3d_to_mesh_error.hpp>
#include <robot_calibration/cost_functions/chain3d_to_plane_error.hpp>
#include <robot_calibration/cost_functions/plane_to_plane_error.hpp>
#include <robot_calibration/cost_functions/outrageous_error.hpp>
#include <robot_calibration/models/camera2d.hpp>
#include <robot_calibration/models/camera3d.hpp>
#include <robot_calibration/models/chain3d.hpp>
#include <string>
#include <map>

namespace robot_calibration
{

Optimizer::Optimizer(const std::string& robot_description) :
  num_params_(0),
  num_residuals_(0)
{
  model_ = std::make_shared<urdf::Model>();
  if (!model_->initString(robot_description))
    std::cerr << "Failed to parse URDF." << std::endl;

  // Maintain consistent offset parser so we hold onto offsets
  offsets_.reset(new OptimizationOffsets());

  // Create a mesh loader
  mesh_loader_.reset(new MeshLoader(model_));
}

Optimizer::~Optimizer()
{
}

int Optimizer::optimize(OptimizationParams& params,
                        std::vector<robot_calibration_msgs::msg::CalibrationData> data,
                        rclcpp::Logger& logger,
                        bool progress_to_stdout)
{
  // Load KDL from URDF
  if (!kdl_parser::treeFromUrdfModel(*model_, tree_))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    return -1;
  }

  // Create models
  for (size_t i = 0; i < params.models.size(); ++i)
  {
    if (params.models[i].type == "chain3d")
    {
      RCLCPP_INFO_STREAM(logger, "Creating chain '" << params.models[i].name << "' from " <<
                                                       params.base_link << " to " <<
                                                       params.models[i].frame);
      Chain3dModel* model = new Chain3dModel(params.models[i].name, tree_, params.base_link, params.models[i].frame);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "camera3d")
    {
      RCLCPP_INFO_STREAM(logger, "Creating camera3d '" << params.models[i].name << "' in frame " <<
                                                          params.models[i].frame);
      std::string param_name = params.models[i].param_name;
      if (param_name == "")
      {
        // Default to same name as sensor
        param_name = params.models[i].name;
      }
      Camera3dModel* model = new Camera3dModel(params.models[i].name, param_name, tree_, params.base_link, params.models[i].frame);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "camera2d")
    {
      RCLCPP_INFO_STREAM(logger, "Creating camera2d '" << params.models[i].name << "' in frame " <<
                                                          params.models[i].frame);
      std::string param_name = params.models[i].param_name;
      if (param_name == "")
      {
        // Default to same name as sensor
        param_name = params.models[i].name;
      }
      Camera2dModel* model = new Camera2dModel(params.models[i].name, param_name, tree_, params.base_link, params.models[i].frame);
      models_[params.models[i].name] = model;
    }
    else
    {
      RCLCPP_ERROR(logger, "Unknown model type: %s", params.models[i].type.c_str());
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
      RCLCPP_ERROR_STREAM(logger, "Error setting initial value for " <<
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
      if (params.error_blocks[j]->type == "chain3d_to_chain3d")
      {
        // This error block can process data generated by the LedFinder,
        // CheckboardFinder, or any other finder that can sample the pose
        // of one or more data points that are connected at a constant offset
        // from a link a kinematic chain (the "arm").
        auto p = std::dynamic_pointer_cast<OptimizationParams::Chain3dToChain3dParams>(params.error_blocks[j]);
        std::string a_name = p->model_a;
        std::string b_name = p->model_b;

        // Do some basic error checking for bad params
        if (a_name == "" || b_name == "" || a_name == b_name)
        {
          RCLCPP_ERROR(logger, "chain3d_to_chain3d improperly configured: model_a and model_b params must be set!");
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
      else if (params.error_blocks[j]->type == "chain3d_to_plane")
      {
        // This error block can process data generated by the PlaneFinder
        auto p = std::dynamic_pointer_cast<OptimizationParams::Chain3dToPlaneParams>(params.error_blocks[j]);
        std::string chain_name = p->model;

        // Do some basic error checking for bad params
        if (chain_name == "")
        {
          RCLCPP_ERROR(logger, "chain3d_to_plane improperly configured: model param must be set!");
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
                                 p->a,
                                 p->b,
                                 p->c,
                                 p->d,
                                 p->scale);

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
      else if (params.error_blocks[j]->type == "chain3d_to_mesh")
      {
        // This error block can process data generated by the RobotFinder
        auto p = std::dynamic_pointer_cast<OptimizationParams::Chain3dToMeshParams>(params.error_blocks[j]);
        std::string chain_name = p->model;

        // Do some basic error checking for bad params
        if (chain_name == "")
        {
          RCLCPP_ERROR(logger, "chain3d_to_mesh improperly configured: model param must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], chain_name))
          continue;

        // Get the mesh
        MeshPtr mesh = mesh_loader_->getCollisionMesh(p->link_name);
        if (!mesh)
        {
          RCLCPP_ERROR(logger, "chain3d_to_mesh improperly configured: cannot load mesh for %s", p->link_name.c_str());
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
      else if (params.error_blocks[j]->type == "chain3d_to_camera2d")
      {
        // This error block can process data generated by the CheckerboardFinder2d,
        auto p = std::dynamic_pointer_cast<OptimizationParams::Chain3dToCamera2dParams>(params.error_blocks[j]);

        // Do some basic error checking for bad params
        if (p->model_3d == "" || p->model_2d == "")
        {
          RCLCPP_ERROR(logger, "chain3d_to_camera2d improperly configured: model_3d and model_2d params must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], p->model_3d) || !hasSensor(data[i], p->model_2d))
        {
          continue;
        }

        // Have to cast our Camera2d model
        auto camera_model = dynamic_cast<Camera2dModel*>(models_[p->model_2d]);
        if (!camera_model)
        {
          RCLCPP_ERROR(logger, "camera2d model is improperly specified");
          return 0;
        }

        // Create the block
        ceres::CostFunction * cost = Chain3dToCamera2d::Create(models_[p->model_3d],
                                                               camera_model,
                                                               p->scale,
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
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 2); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(2*k + 0)];
          std::cout << std::endl << "  y: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 2); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(2*k + 1)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL,  // squared loss
                                  free_params);
      }
      else if (params.error_blocks[j]->type == "plane_to_plane")
      {
        // This error block can process data generated by the PlaneFinder,
        // CheckerboardFinder, or any other finder that returns a series of
        // planar points.
        auto p = std::dynamic_pointer_cast<OptimizationParams::PlaneToPlaneParams>(params.error_blocks[j]);
        std::string a_name = p->model_a;
        std::string b_name = p->model_b;

        // Do some basic error checking for bad params
        if (a_name == "" || b_name == "" || a_name == b_name)
        {
          RCLCPP_ERROR(logger, "plane_to_plane improperly configured: model_a and model_a params must be set!");
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
                                    p->normal_scale,
                                    p->offset_scale);

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
      else if (params.error_blocks[j]->type == "outrageous")
      {
        // Outrageous error block requires no particular sensors, add to every sample
        auto p = std::dynamic_pointer_cast<OptimizationParams::OutrageousParams>(params.error_blocks[j]);
        problem->AddResidualBlock(
          OutrageousError::Create(offsets_.get(),
                                  p->param,
                                  p->joint_scale,
                                  p->position_scale,
                                  p->rotation_scale),
          NULL, // squared loss
          free_params);
      }
      else
      {
        RCLCPP_ERROR(logger, "Unknown error block: %s", params.error_blocks[j]->type.c_str());
        return 0;
      }
    }
  }

  // Setup the actual optimization
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1e-10;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = params.max_num_iterations;
  options.minimizer_progress_to_stdout = progress_to_stdout;

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

std::vector<std::string> Optimizer::getCameraNames()
{
  std::vector<std::string> camera_names;
  for (auto it = models_.begin(); it != models_.end(); ++it)
  {
    if (it->second->getType() == "Camera3dModel")
    {
       camera_names.push_back(it->first);
    }
  }
  return camera_names;
}

}  // namespace robot_calibration
