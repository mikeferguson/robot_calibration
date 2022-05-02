#include <boost/algorithm/string.hpp>
#include <urdf/model.h>
#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/models/chain.h>  // for rotation functions
#include <gtest/gtest.h>

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

std::string robot_description_updated =
"<?xml version=\"1.0\" ?>\n"
"<robot name=\"test\">\n"
"  <link name=\"link_0\" />\n"
"  <joint name=\"first_joint\" type=\"fixed\">\n"
"    <origin rpy=\"0 0 0\" xyz=\"1 1 1\" />\n"
"    <parent link=\"link_0\" />\n"
"    <child link=\"link_1\" />\n"
"  </joint>\n"
"  <link name=\"link_1\" />\n"
"  <joint name=\"second_joint\" type=\"revolute\">\n"
"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\" />\n"
"    <axis xyz=\"0 0 1\" />\n"
"    <limit effort=\"30\" lower=\"-1.57\" upper=\"1.57\" velocity=\"0.524\" />\n"
"    <parent link=\"link_1\" />\n"
"    <child link=\"link_2\" />\n"
"    <calibration rising=\"0.245\" />\n"
"  </joint>\n"
"  <link name=\"link_2\" />\n"
"  <joint name=\"third_joint\" type=\"fixed\">\n"
"    <origin rpy=\"1.57000000 -1.50000000 0.00000000\" xyz=\"0.00000000 0.00000000 0.05260000\" />\n"
"    <parent link=\"link_2\" />\n"
"    <child link=\"link_3\" />\n"
"  </joint>\n"
"  <link name=\"link_3\" />\n"
"</robot>";

TEST(CalibrationOffsetParserTests, test_urdf_update)
{
  robot_calibration::CalibrationOffsetParser p;

  p.add("second_joint");
  p.addFrame("third_joint", true, true, true, true, true, true);

  double params[7] = {0.245, 0, 0, 0, 0, 0, 0};

  // set angles
  KDL::Rotation r = KDL::Rotation::RPY(1.57, 0, 0);
  robot_calibration::axis_magnitude_from_rotation(r, params[4], params[5], params[6]);

  p.update(params);

  std::string s = p.updateURDF(robot_description);
  
  // google test fails if we give it all of robot_description_updated, so break this up
  std::vector<std::string> s_pieces;
  std::vector<std::string> robot_pieces;

  boost::split(s_pieces, s, boost::is_any_of("\n"));
  boost::split(robot_pieces, robot_description_updated, boost::is_any_of("\n"));

  for (size_t i = 0; i < robot_pieces.size(); ++i)
  {
    ASSERT_STREQ(robot_pieces[i].c_str(), s_pieces[i].c_str());
  }
}

TEST(CalibrationOffsetParserTests, test_multi_step)
{
  robot_calibration::CalibrationOffsetParser p;

  p.add("first_step_joint1");
  p.add("first_step_joint2");

  double params[2] = {0.245, 0.44};
  p.update(params);

  EXPECT_EQ(0.245, p.get("first_step_joint1"));
  EXPECT_EQ(0.44, p.get("first_step_joint2"));
  EXPECT_EQ((size_t) 2, p.size());

  // Reset num of free params
  p.reset();
  EXPECT_EQ((size_t) 0, p.size());

  // Add a new one for second step
  p.add("second_step_joint1");
  EXPECT_EQ((size_t) 1, p.size());

  params[0] *= 2.0;
  p.update(params);

  EXPECT_EQ(0.245, p.get("first_step_joint1"));
  EXPECT_EQ(0.44, p.get("first_step_joint2"));
  EXPECT_EQ(0.49, p.get("second_step_joint1"));

  // Reset num of free params
  p.reset();
  EXPECT_EQ((size_t) 0, p.size());

  // Reuse a param for a third step
  p.add("first_step_joint1");
  EXPECT_EQ((size_t) 1, p.size());

  params[0] *= 2.0;
  p.update(params);

  EXPECT_EQ(0.98, p.get("first_step_joint1"));
  EXPECT_EQ(0.44, p.get("first_step_joint2"));
  EXPECT_EQ(0.49, p.get("second_step_joint1"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
