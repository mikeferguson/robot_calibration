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

#include <robot_calibration/capture/feature_finder_loader.h>
#include <gtest/gtest.h>

TEST(FeatureFinderLoaderTests, test_feature_finder_loader)
{
  ros::NodeHandle nh("~");
  robot_calibration::FeatureFinderLoader loader;
  robot_calibration::FeatureFinderMap features;
  bool result = loader.load(nh, features);

  EXPECT_EQ(true, result);
  EXPECT_EQ(2, features.size());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_finder_loader_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
