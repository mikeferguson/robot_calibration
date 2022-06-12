/*
 * Copyright (C) 2022 Michael Ferguson
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

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <robot_calibration/mesh_loader.h>

namespace robot_calibration
{

MeshLoader::MeshLoader(std::shared_ptr<urdf::Model> model) : model_(model)
{
}

MeshPtr MeshLoader::getCollisionMesh(const std::string& link_name)
{
  // See if we have already loaded the mesh
  for (size_t i = 0; i < link_names_.size(); ++i)
  {
    if (link_names_[i] == link_name)
    {
      return meshes_[i];
    }
  }

  // Find the mesh resource path
  urdf::LinkConstSharedPtr link = model_->getLink(link_name);
  if (!link)
  {
    //ROS_ERROR("Cannot find %s in URDF", link_name.c_str());
    return MeshPtr();
  }

  if (!link->collision->geometry)
  {
    //ROS_ERROR("%s does not have collision geometry description.", link_name.c_str());
    return MeshPtr();
  }

  if (link->collision->geometry->type != urdf::Geometry::MESH)
  {
    //ROS_ERROR("%s does not have mesh geometry", link_name.c_str());
    return MeshPtr();
  }

  // This is the resource path (package://path/x.mesh)
  std::string mesh_path = (dynamic_cast<urdf::Mesh*>(link->collision->geometry.get()))->filename;

  // Get the scale
  Eigen::Vector3d scale((dynamic_cast<urdf::Mesh*>(link->collision->geometry.get()))->scale.x,
                        (dynamic_cast<urdf::Mesh*>(link->collision->geometry.get()))->scale.y,
                        (dynamic_cast<urdf::Mesh*>(link->collision->geometry.get()))->scale.z);

  MeshPtr mesh(shapes::createMeshFromResource(mesh_path, scale));
  link_names_.push_back(link_name);
  meshes_.push_back(mesh);

  //ROS_INFO("Loaded %s with %u vertices", mesh_path.c_str(), mesh->vertex_count);

  //Eigen::Quaterniond quat();
  Eigen::Matrix3d rotation = Eigen::Quaterniond(link->collision->origin.rotation.w,
                                                link->collision->origin.rotation.x,
                                                link->collision->origin.rotation.y,
                                                link->collision->origin.rotation.z).toRotationMatrix();
  Eigen::Vector3d translation(link->collision->origin.position.x,
                              link->collision->origin.position.y,
                              link->collision->origin.position.z);

  // Transform to proper location
  for (size_t v = 0; v < mesh->vertex_count; ++v)
  {
    Eigen::Vector3d p(mesh->vertices[(3 * v) + 0],
                      mesh->vertices[(3 * v) + 1],
                      mesh->vertices[(3 * v) + 2]);

    p = (rotation * p) + translation;

    mesh->vertices[(3 * v) + 0] = p(0);
    mesh->vertices[(3 * v) + 1] = p(1);
    mesh->vertices[(3 * v) + 2] = p(2);
  }

  return mesh;
}

}  // namespace robot_calibration
