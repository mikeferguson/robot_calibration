/*
 * Copyright (C) 2020 Michael Ferguson
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

#include <vector>

#include <robot_calibration/ceres/magnetometer_error.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/MagneticField.h>

class MagnetometerCapture
{
public:
  MagnetometerCapture(const std::string& topic, ros::NodeHandle& nh)
  {
    subscriber_ = nh.subscribe(topic,
                               1,
                               &MagnetometerCapture::callback,
                               this);
  }

  void getData(std::vector<sensor_msgs::MagneticField>& data)
  {
    data = data_;
  }

private:
  void callback(const sensor_msgs::MagneticFieldConstPtr& msg)
  {
    data_.push_back(*msg);
  }

  ros::Subscriber subscriber_;
  std::vector<sensor_msgs::MagneticField> data_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv,"magnetometer_calibration");
  ros::NodeHandle nh("~");

  /*
   * "hard iron" offset is the result of nearby permanent ferromagnetic
   * elements (for instance, components on the PCB.
   *
   * "soft iron" effect is induced by the geomagnetic field onto normally
   * unmagnetized ferromagnetic elements.
   */
  bool soft_iron = false;
  nh.getParam("soft_iron", soft_iron);
  if (soft_iron)
  {
    ROS_ERROR("Soft Iron Calibration Is Not Yet Available.");
  }

  /*
   * Speed and duration to rotate the robot.
   * Setting either of these to 0.0 will skip rotation.
   */
  double rotation_velocity = 0.2;   // rad/s
  double rotation_duration = 60.0;  // seconds
  nh.getParam("rotation_velocity", rotation_velocity);
  nh.getParam("rotation_duration", rotation_duration);

  /*
   * If this is a handheld IMU, you can manually
   * rotate the sensor and press ENTER when done.
   */
  bool rotation_manual = false;
  nh.getParam("rotation_manual", rotation_manual);

  // Name of bagfile to create and/or use
  std::string bag_file_name = "/tmp/magnetometer_calibration.bag";
  nh.getParam("bag_file_name", bag_file_name);

  // Determine topic name (including remapping)
  std::string mag_topic_name = "/imu/mag";
  //mag_topic_name = ros::names::resolve(mag_topic_name);

  // Get calibration data
  std::vector<sensor_msgs::MagneticField> data;
  if (rotation_velocity <= 0.0 && rotation_duration <= 0.0 && !rotation_manual)
  {
    // Load calibration data from bag file
    rosbag::Bag bag;
    try
    {
      bag.open(bag_file_name, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException)
    {
      ROS_FATAL_STREAM("Cannot open " << bag_file_name);
      return -1;
    }
    rosbag::View bag_view(bag, rosbag::TopicQuery(mag_topic_name));

    for (auto m : bag_view)
    {
      sensor_msgs::MagneticField::ConstPtr msg = m.instantiate<sensor_msgs::MagneticField>();
      if (msg == NULL)
      {
        // Try as Vector3Stamped
        geometry_msgs::Vector3Stamped::ConstPtr vec = m.instantiate<geometry_msgs::Vector3Stamped>();
        if (vec != NULL)
        {
          sensor_msgs::MagneticField field;
          field.header = vec->header;
          field.magnetic_field.x = vec->vector.x;
          field.magnetic_field.y = vec->vector.y;
          field.magnetic_field.z = vec->vector.z;
          data.push_back(field);
        }
      }
      else
      {
        data.push_back(*msg);
      }
    }

    ROS_INFO_STREAM("Loaded bag file with " << data.size() << " samples");
  }
  else
  {
    // Create calibration (and bag file)
    MagnetometerCapture capture(mag_topic_name, nh);

    if (rotation_manual)
    {
      // Wait for keypress
      ROS_INFO("Rotate the sensor to many orientations.");
      ROS_INFO("Press key when done rotating");
      std::string throwaway;
      std::getline(std::cin, throwaway);
    }
    else
    {
      // Rotate the robot
      ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

      geometry_msgs::Twist msg;
      msg.angular.z = rotation_velocity;

      ROS_INFO_STREAM("Rotating robot for " << rotation_duration << " seconds.");

      ros::Time start = ros::Time::now();
      while ((ros::Time::now() - start).toSec() < rotation_duration)
      {
        pub.publish(msg);
        ros::Duration(0.1).sleep(); 
      }

      msg.angular.z = 0.0;
      pub.publish(msg);

      ROS_INFO("Done rotating robot.");
    }

    // Get data
    capture.getData(data);

    // Save data to bag file
    ROS_INFO_STREAM("Saving bag file with " << data.size() << " samples.");
    rosbag::Bag bag;
    for (auto msg : data)
    {
      bag.write(mag_topic_name, msg.header.stamp, msg);
    }
  }

  // Setup free parameters
  size_t num_params = 4;
  if (soft_iron)
  {
    num_params += 6;
  }
  double* free_params = new double[num_params];
  // Set initial estimate of magnetic field strength
  free_params[0] = 0.45;
  // Setup initial estimates of hard iron offsets (bias)
  {
    double bias_x = 0.0;
    double bias_y = 0.0;
    double bias_z = 0.0;
    for (auto msg : data)
    {
      bias_x += msg.magnetic_field.x;
      bias_y += msg.magnetic_field.y;
      bias_z += msg.magnetic_field.z;
    }
    bias_x /= data.size();
    bias_y /= data.size();
    bias_z /= data.size();

    free_params[1] = bias_x;
    free_params[2] = bias_y;
    free_params[3] = bias_z;

    ROS_INFO_STREAM("Initial estimate for hard iron offsets: [" <<
                    bias_x << ", " <<
                    bias_y << ", " <<
                    bias_z << "]");
  }

  // Setup error blocks
  ceres::Problem* problem = new ceres::Problem();
  ceres::TrivialLoss loss;
  for (auto msg : data)
  {
    auto m = msg.magnetic_field;
    if (soft_iron)
    {

    }
    else
    {
      problem->AddResidualBlock(
        HardIronOffsetError::Create(m.x, m.y, m.z),
        &loss,  // squared loss
        free_params);
    }
  }

  // Run calibration
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1e-10;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  ROS_INFO_STREAM(summary.BriefReport());

  // Save results
  ROS_INFO_STREAM("Estimated total magnetic field: " << free_params[0] << "T");
  ROS_INFO("You can compare to expected values from");
  ROS_INFO("  https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm");

  std::cout << "mag_bias_x: " << free_params[1] << std::endl;
  std::cout << "mag_bias_y: " << free_params[2] << std::endl;
  std::cout << "mag_bias_z: " << free_params[3] << std::endl;
  delete[] free_params;

  return 0;
}
