#include <gtest/gtest.h>
#include <robot_calibration/util/poses_from_yaml.hpp>

TEST(PosesFromYamlTests, test_yaml_loading)
{
  std::vector<robot_calibration_msgs::msg::CaptureConfig> poses;
  EXPECT_TRUE(robot_calibration::getPosesFromYaml("poses_from_yaml_tests.yaml", poses));

  EXPECT_EQ(3, static_cast<int>(poses.size()));

  EXPECT_EQ("j1", poses[0].joint_states.name[0]);
  EXPECT_EQ("j2", poses[0].joint_states.name[1]);
  EXPECT_EQ("j3", poses[0].joint_states.name[2]);
  EXPECT_EQ(0.1, poses[0].joint_states.position[0]);
  EXPECT_EQ(0.2, poses[0].joint_states.position[1]);
  EXPECT_EQ(0.3, poses[0].joint_states.position[2]);
  EXPECT_EQ("feature1", poses[0].features[0]);
  EXPECT_EQ("feature2", poses[0].features[1]);

  EXPECT_EQ("j1", poses[1].joint_states.name[0]);
  EXPECT_EQ("j2", poses[1].joint_states.name[1]);
  EXPECT_EQ("j3", poses[1].joint_states.name[2]);
  EXPECT_EQ(0.4, poses[1].joint_states.position[0]);
  EXPECT_EQ(0.5, poses[1].joint_states.position[1]);
  EXPECT_EQ(0.6, poses[1].joint_states.position[2]);
  EXPECT_TRUE(poses[1].features.empty());

  EXPECT_EQ("j4", poses[2].joint_states.name[0]);
  EXPECT_EQ("j5", poses[2].joint_states.name[1]);
  EXPECT_EQ("j6", poses[2].joint_states.name[2]);
  EXPECT_EQ(0.7, poses[2].joint_states.position[0]);
  EXPECT_EQ(0.8, poses[2].joint_states.position[1]);
  EXPECT_EQ(0.9, poses[2].joint_states.position[2]);
  EXPECT_TRUE(poses[2].features.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
