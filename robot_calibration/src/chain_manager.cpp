/*
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

#include <robot_calibration/capture/chain_manager.h>

namespace robot_calibration
{

ChainManager::ChainManager(ros::NodeHandle& nh, double wait_time)
{
  // We cannot do much without some kinematic chains
  if (!nh.hasParam("chains"))
  {
    // TODO raise error
  }

  // Get chains
  XmlRpc::XmlRpcValue chains;
  nh.getParam("chains", chains);
  ROS_ASSERT(chains.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // Construct each chain to manage
  for (size_t i = 0; i < chains.size(); ++i)
  {
    std::string name, topic;
    name = static_cast<std::string>(chains[i]["name"]);
    topic = static_cast<std::string>(chains[i]["topic"]);

    boost::shared_ptr<ChainController> controller(new ChainController(name, topic));

    for (size_t j = 0; j < chains[i]["joints"].size(); ++j)
    {
      controller->joint_names.push_back(static_cast<std::string>(chains[i]["joints"][j]));
    }

    ROS_INFO("Waiting for %s...", topic.c_str());
    controller->client.waitForServer(ros::Duration(wait_time));

    controllers_.push_back(controller);
  }

  // Parameter to set movement time
  nh.param<double>("duration", duration_, 5.0);

  subscriber_ = nh.subscribe("/joint_states", 1, &ChainManager::stateCallback, this);
}

void ChainManager::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  boost::mutex::scoped_lock lock(state_mutex_);
  // Update each joint based on message
  for (size_t msg_j = 0; msg_j < msg->name.size(); msg_j++)
  {
    size_t state_j;
    for (state_j = 0; state_j < state_.name.size(); state_j++)
    {
      if (state_.name[state_j] == msg->name[msg_j])
      {
        state_.position[state_j] = msg->position[msg_j];
        break;
      }
    }
    if (state_j == state_.name.size())
    {
      // New joint
      state_.name.push_back(msg->name[msg_j]);
      state_.position.push_back(msg->position[msg_j]);
    }
  }
}

bool ChainManager::getState(sensor_msgs::JointState* state)
{
  boost::mutex::scoped_lock lock(state_mutex_);
  *state = state_;
}

trajectory_msgs::JointTrajectoryPoint
ChainManager::makePoint(const sensor_msgs::JointState& state, const std::vector<std::string>& joints)
{
  trajectory_msgs::JointTrajectoryPoint p;
  for (size_t i = 0; i < joints.size(); ++i)
  {
    for (size_t j = 0; j < state.name.size(); ++j)
    {
      if (joints[i] == state.name[j])
      {
        p.positions.push_back(state.position[j]);
        break;
      }
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
  // Split into different controllers
  for(size_t i = 0; i < controllers_.size(); ++i)
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = controllers_[i]->joint_names;

    trajectory_msgs::JointTrajectoryPoint p = makePoint(state, controllers_[i]->joint_names);
    p.time_from_start = ros::Duration(duration_);
    goal.trajectory.points.push_back(p);
    goal.goal_time_tolerance = ros::Duration(1.0);

    // Call actions
    controllers_[i]->client.sendGoal(goal);
  }

  // Wait for results
  for (size_t i = 0; i < controllers_.size(); ++i)
  {
    // TODO: catch errors with clients
    controllers_[i]->client.waitForResult(ros::Duration(duration_*1.5));
  }

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

      for (size_t i = 0; i < controllers_.size(); ++i)
      {
        for (size_t k = 0; k < controllers_[i]->joint_names.size(); ++k)
        {
          if (controllers_[i]->joint_names[k] == state.name[j])
          {
            settled = false;
            break;
          }
        }
      }

      // If at least one joint is not settled, break out of this for loop
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

std::vector<std::string> ChainManager::getChains()
{
  std::vector<std::string> chains;
  for (int i = 0; i < controllers_.size(); ++i)
  {
    chains.push_back(controllers_[i]->chain_name);
  }
  return chains;
}

std::vector<std::string> ChainManager::getChainJointNames(
  const std::string& chain_name)
{
  for (int i = 0; i < controllers_.size(); ++i)
  {
    if (controllers_[i]->chain_name == chain_name)
      return controllers_[i]->joint_names;
  }
  std::vector<std::string> empty;
  return empty;
}

}  // namespace robot_calibration
