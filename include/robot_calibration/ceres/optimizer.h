/*
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

#ifndef ROBOT_CALIBRATION_CERES_OPTIMIZER_H
#define ROBOT_CALIBRATION_CERES_OPTIMIZER_H

#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_calibration/CalibrationData.h>

#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/ceres/camera3d_to_arm_error.h>
#include <robot_calibration/ceres/data_functions.h>
#include <robot_calibration/ceres/outrageous_error.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

namespace robot_calibration
{

/** \brief Class to do optimization. */
class Optimizer
{
public:
  /** \brief Standard constructor */
  Optimizer(std::string robot_description, std::string root_frame, std::string led_frame) :
    root_frame_(root_frame), led_frame_(led_frame)
  {
    if (!model_.initString(robot_description))
      std::cerr << "Failed to parse URDF." << std::endl;
  }

  virtual ~Optimizer()
  {
    if (free_params_)
      delete[] free_params_;
    if (problem_)
      delete problem_;
  }

  /**
   *  \brief Run optimization.
   *  \param data The data to be used for the optimization. Typically parsed
   *         from bag file, or loaded over some topic subscriber.
   *  \param progress_to_stdout If true, Ceres optimizer will output info to
   *         stdout.
   */
  int optimize(std::vector<robot_calibration::CalibrationData> data,
               bool progress_to_stdout = false)
  {
    // Load KDL from URDF
    if (!kdl_parser::treeFromUrdfModel(model_, tree_))
    {
      std::cerr << "Failed to construct KDL tree" << std::endl;
      return -1;
    }

    // Create models
    arm_model_ = new ChainModel(tree_, root_frame_, led_frame_);
    camera_model_ = new Camera3dModel(tree_, root_frame_, data[0].rgbd_observations[0].header.frame_id);

    // Setup  parameters to calibrate
    offsets_ = new CalibrationOffsetParser();
    //offsets_.add("torso_lift_joint");
    offsets_->add("shoulder_pan_joint");
    offsets_->add("shoulder_lift_joint");
    offsets_->add("upperarm_roll_joint");
    offsets_->add("elbow_flex_joint");
    offsets_->add("forearm_roll_joint");
    offsets_->add("wrist_flex_joint");
    offsets_->add("wrist_roll_joint");
    offsets_->add("head_pan_joint");
    offsets_->add("head_tilt_joint");
    // Neck is typically inexact - calibrate position & pitch
    offsets_->addFrame("head_pan_joint", true, true, true, false, true, false);
    // Calibrate 6dof pose of camera
    offsets_->addFrame("head_camera_rgb_joint", true, true, true, true, true, true);
    offsets_->add("camera_fx");
    offsets_->add("camera_fy");
    offsets_->add("camera_cx");
    offsets_->add("camera_cy");
    // Calibrate model of Primesense sensor
    offsets_->add("camera_z_offset");
    offsets_->add("camera_z_scaling");
    // Support checkerboards
    if (data[0].world_observations[0].header.frame_id.compare("checkerboard") == 0)
      offsets_->addFrame("checkerboard", true, true, true, true, true, true);

    // Allocate space
    free_params_ = new double[offsets_->size()];
    for (int i = 0; i < offsets_->size(); ++i)
      free_params_[i] = 0.0;

    // Houston, we have a problem...
    problem_ = new ceres::Problem();

    // For each observation:
    for (size_t i = 0; i < data.size(); ++i)
    {
      ceres::CostFunction * cost = Camera3dToArmError::Create(camera_model_, arm_model_, offsets_, data[i]);

      if (progress_to_stdout)
      {
        double ** params = new double*[1];
        params[0] = free_params_;
        double * residuals = new double[data[i].rgbd_observations.size() * 3];

        cost->Evaluate(params, residuals, NULL);
        std::cout << "INITIAL COST (" << i << ")" << std::endl << "  x: ";
        for (size_t k = 0; k < data[i].rgbd_observations.size(); ++k)
          std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 0)];
        std::cout << std::endl << "  y: ";
        for (size_t k = 0; k < data[i].rgbd_observations.size(); ++k)
          std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 1)];
        std::cout << std::endl << "  z: ";
        for (size_t k = 0; k < data[i].rgbd_observations.size(); ++k)
          std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 2)];
        std::cout << std::endl << std::endl;
      }

      problem_->AddResidualBlock(cost,
                                 NULL /* squared loss */,
                                 free_params_);
    }

    problem_->AddResidualBlock(OutrageousError::Create(offsets_, "head_camera_rgb_joint", 0.0, 0.1, 0.1),
                               NULL, // squared loss
                               free_params_);
    problem_->AddResidualBlock(OutrageousError::Create(offsets_, "head_pan_joint", 0.1, 0.1, 0.1),
                               NULL,  // squared loss
                               free_params_);
    problem_->AddResidualBlock(OutrageousError::Create(offsets_, "elbow_flex_joint", 0.1, 0.0, 0.0),
                               NULL,  // squared loss
                               free_params_);
    problem_->AddResidualBlock(OutrageousError::Create(offsets_, "forearm_roll_joint", 0.1, 0.0, 0.0),
                               NULL,  // squared loss
                               free_params_);
    problem_->AddResidualBlock(OutrageousError::Create(offsets_, "wrist_flex_joint", 0.1, 0.0, 0.0),
                               NULL,  // squared loss
                               free_params_);
    problem_->AddResidualBlock(OutrageousError::Create(offsets_, "wrist_roll_joint", 0.1, 0.0, 0.0),
                               NULL,  // squared loss
                               free_params_);

    // Setup the actual optimization
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.function_tolerance = 1e-10;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = progress_to_stdout;
    //options.use_nonmonotonic_steps = true;

    if (progress_to_stdout)
      std::cout << "\nSolver output:" << std::endl;
    summary_ = new ceres::Solver::Summary();
    ceres::Solve(options, problem_, summary_);
    if (progress_to_stdout)
      std::cout << "\n" << summary_->BriefReport() << std::endl;

    // TODO output stats
    if (progress_to_stdout)
    {
      CalibrationOffsetParser no_offsets;
      offsets_->update(free_params_);
      for (size_t i = 0; i < data.size(); ++i)
      {
        std::cout << "Sample " << i << std::endl;
        printSimpleDistanceError(arm_model_, camera_model_, &no_offsets, offsets_, data[i]);
        printComparePoints(arm_model_, camera_model_, &no_offsets, offsets_, data[i]);
      }
    }

    // Note: the error blocks will be managed by scoped_ptr in cost functor
    //       which takes ownership, and so we do not need to delete them here

    return 0;
  }

  /** \brief Returns the summary of the optimization last run. */
  ceres::Solver::Summary* summary()
  {
    return summary_;
  }

  CalibrationOffsetParser& getOffsets()
  {
    return *offsets_;
  }

private:
  urdf::Model model_;
  std::string root_frame_;
  std::string led_frame_;
  KDL::Tree tree_;

  ChainModel * arm_model_;
  Camera3dModel * camera_model_;

  int num_free_params_;
  double * free_params_;
  CalibrationOffsetParser* offsets_;

  ceres::Problem* problem_;
  ceres::Solver::Summary* summary_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_OPTIMIZER_H