#include <robot_calibration/models/chain.h>
#include <gtest/gtest.h>

using robot_calibration::rotation_from_axis_magnitude;
using robot_calibration::axis_magnitude_from_rotation;

TEST(RotationTests, rotationsOK)
{
  KDL::Rotation one_radian_axis_x1 = KDL::Rotation::RPY(1.0, 0.0, 0.0);
  KDL::Rotation one_radian_axis_x2 = rotation_from_axis_magnitude(1.0, 0.0, 0.0);

  EXPECT_EQ(one_radian_axis_x1.UnitX(), one_radian_axis_x2.UnitX());
  EXPECT_EQ(one_radian_axis_x1.UnitY(), one_radian_axis_x2.UnitY());
  EXPECT_EQ(one_radian_axis_x1.UnitZ(), one_radian_axis_x2.UnitZ());

  KDL::Rotation neg_radian_axis_x1 = KDL::Rotation::RPY(-1.0, 0.0, 0.0);
  KDL::Rotation neg_radian_axis_x2 = rotation_from_axis_magnitude(-1.0, 0.0, 0.0);

  EXPECT_EQ(neg_radian_axis_x1.UnitX(), neg_radian_axis_x2.UnitX());
  EXPECT_EQ(neg_radian_axis_x1.UnitY(), neg_radian_axis_x2.UnitY());
  EXPECT_EQ(neg_radian_axis_x1.UnitZ(), neg_radian_axis_x2.UnitZ());

  KDL::Rotation one_radian_axis_z1 = KDL::Rotation::RPY(0.0, 0.0, 1.0);
  KDL::Rotation one_radian_axis_z2 = rotation_from_axis_magnitude(0.0, 0.0, 1.0);

  EXPECT_EQ(one_radian_axis_z1.UnitX(), one_radian_axis_z2.UnitX());
  EXPECT_EQ(one_radian_axis_z1.UnitY(), one_radian_axis_z2.UnitY());
  EXPECT_EQ(one_radian_axis_z1.UnitZ(), one_radian_axis_z2.UnitZ());

  double x, y, z;
  axis_magnitude_from_rotation(one_radian_axis_x1, x, y, z);
  EXPECT_DOUBLE_EQ(1.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, z);

  axis_magnitude_from_rotation(neg_radian_axis_x1, x, y, z);
  EXPECT_DOUBLE_EQ(-1.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, z);

  axis_magnitude_from_rotation(one_radian_axis_z1, x, y, z);
  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(1.0, z);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}