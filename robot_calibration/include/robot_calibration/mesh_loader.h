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

#ifndef ROBOT_CALIBRATION_MESH_LOADER_H
#define ROBOT_CALIBRATION_MESH_LOADER_H

#include <memory>
#include <string>
#include <vector>

#include <geometric_shapes/shape_operations.h>
#include <urdf/model.h>

namespace robot_calibration
{

using MeshPtr = std::shared_ptr<shapes::Mesh>;

class MeshLoader
{
public:
  MeshLoader(std::shared_ptr<urdf::Model> model);

  /**
   * @brief Get the collision mesh associated with a link in a URDF.
   */
  MeshPtr getCollisionMesh(const std::string& link_name);

private:
  std::shared_ptr<urdf::Model> model_;
  std::vector<std::string> link_names_;
  std::vector<MeshPtr> meshes_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_MESH_LOADER_H
