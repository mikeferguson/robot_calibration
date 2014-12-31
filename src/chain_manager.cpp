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

#include <robot_calibration/capture/chain_manager.h>

namespace robot_calibration
{

// TODO: need mutex here?
void ChainManager::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  state_ = *msg;
}

bool ChainManager::getState(sensor_msgs::JointState* state)
{
  *state = state_;
}

trajectory_msgs::JointTrajectoryPoint
ChainManager::makePoint(const sensor_msgs::JointState& state, const std::vector<std::string> joints)
{
  trajectory_msgs::JointTrajectoryPoint p;
  for (size_t i = 0; i < joints.size(); ++i)
  {
    for (size_t j = 0; j < state.name.size(); ++j)
      if (joints[i] == state.name[j])
      {
        p.positions.push_back(state.position[j]);
        break;
      }
    p.velocities.push_back(0.0);
    p.accelerations.push_back(0.0);
    if (p.velocities.size() != p.positions.size())
    {
      ROS_ERROR_STREAM("Bad move to state, missing " << joints[i]);
      exit(-1);
    }
  }
  return p;
}

bool ChainManager::moveToState(const sensor_msgs::JointState& state)
{
  // Split into head and arm
  control_msgs::FollowJointTrajectoryGoal head_goal;
  head_goal.trajectory.joint_names = head_joints_;

  trajectory_msgs::JointTrajectoryPoint p = makePoint(state, head_joints_);
  p.time_from_start = ros::Duration(1.0);
  head_goal.trajectory.points.push_back(p);
  head_goal.goal_time_tolerance = ros::Duration(3.0);

  control_msgs::FollowJointTrajectoryGoal arm_goal;
  arm_goal.trajectory.joint_names = arm_joints_;

  p = makePoint(state, arm_joints_);
  p.time_from_start = ros::Duration(3.0);
  arm_goal.trajectory.points.push_back(p);
  arm_goal.goal_time_tolerance = ros::Duration(1.0);

  // Call actions
  head_client_.sendGoal(head_goal);
  arm_client_.sendGoal(arm_goal);

  // Wait for results
  head_client_.waitForResult(ros::Duration(15.0));
  arm_client_.waitForResult(ros::Duration(15.0));

  // TODO: catch errors with clients

  return true;
}

bool ChainManager::waitToSettle()
{
  sensor_msgs::JointState state;
  int stable = 0;

  // TODO: timeout?
  while (true)
  {
    getState(&state);
    bool settled = true;

    // For each joint in state message
    for (size_t j = 0; j < state.name.size(); ++j)
    {
      // Is this joint even a concern?
      if (fabs(state.velocity[j]) < 0.001)
        continue;

      // Is this joint in head?
      for (size_t i = 0; i < head_joints_.size(); ++i)
      {
        if (head_joints_[i] == state.name[j])
        {
          settled = false;
          break;
        }
      }

      // Is this joint in the arm?
      for (size_t i = 0; i < arm_joints_.size(); ++i)
      {
        if (arm_joints_[i] == state.name[j])
        {
          settled = false;
          break;
        }
      }

      // If at least one joint is not settled, break out this for loop
      if (!settled)
        break;
    }

    // If all joints are settled, break out of while loop
    if (settled)
      break;

    ros::spinOnce();
  }

  return true;
}

}  // namespace robot_calibration