/*
 * Copyright (C) 2018-2022 Michael Ferguson
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

#include <robot_calibration/finders/loader.hpp>
#include <gtest/gtest.h>

TEST(FeatureFinderLoaderTests, test_feature_finder_loader)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("feature_finder_loader_tests");
  robot_calibration::FeatureFinderLoader loader;
  robot_calibration::FeatureFinderMap features;
  bool result = loader.load(node, features);

  EXPECT_EQ(true, result);
  EXPECT_EQ(static_cast<size_t>(2), features.size());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
