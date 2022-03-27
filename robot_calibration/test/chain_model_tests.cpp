/*
 * Copyright (C) 2018 Michael Ferguson
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

// Author: Gerardo Puga

#include <gtest/gtest.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>

#include <kdl_parser/kdl_parser.hpp>

namespace {

namespace test {

using robot_calibration::Camera3dModel;
using robot_calibration::ChainModel;

std::string robot_description =
    "<?xml version='1.0' ?>"
    "<robot name='test'>"
    "  <link name='link_0'/>"
    "  <joint name='first_joint' type='fixed'>"
    "    <origin rpy='0 0 0' xyz='1 1 1'/>"
    "    <parent link='link_0'/>"
    "    <child link='link_1'/>"
    "  </joint>"
    "  <link name='link_1'/>"
    "  <joint name='second_joint' type='revolute'>"
    "    <origin rpy='0 0 0' xyz='0 0 0'/>"
    "    <axis xyz='0 0 1'/>"
    "    <limit effort='30' lower='-1.57' upper='1.57' velocity='0.524'/>"
    "    <parent link='link_1'/>"
    "    <child link='link_2'/>"
    "  </joint>"
    "  <link name='link_2'/>"
    "  <joint name='third_joint' type='fixed'>"
    "    <origin rpy='0 -1.5 0' xyz='0 0 0.0526'/>"
    "    <parent link='link_2'/>"
    "    <child link='link_3'/>"
    "  </joint>"
    "  <link name='link_3'/>"
    "</robot>";

TEST(ChainModelTests, BadChainThrows)
{
  KDL::Tree tree;

  ASSERT_TRUE(kdl_parser::treeFromString(robot_description, tree));

  ASSERT_NO_THROW({ auto uut = ChainModel("uut", tree, "link_0", "link_3"); });

  ASSERT_THROW({ auto uut = ChainModel("uut", tree, "link_99", "link_3"); },
               std::runtime_error);

  ASSERT_THROW({ auto uut = ChainModel("uut", tree, "link_0", "link_99"); },
               std::runtime_error);
}

TEST(Camera3dModelTests, BadChainThrows)
{
  KDL::Tree tree;

  ASSERT_TRUE(kdl_parser::treeFromString(robot_description, tree));

  ASSERT_NO_THROW(
      { auto uut = Camera3dModel("uut", "uut", tree, "link_0", "link_3"); });

  ASSERT_THROW({ auto uut = Camera3dModel("uut", "uut", tree, "link_99", "link_3"); },
               std::runtime_error);

  ASSERT_THROW({ auto uut = Camera3dModel("uut", "uut", tree, "link_0", "link_99"); },
               std::runtime_error);
}

};  // namespace test

};  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
