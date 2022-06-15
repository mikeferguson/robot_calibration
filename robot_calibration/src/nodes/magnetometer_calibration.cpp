/*
 * Copyright (C) 2020-2022 Michael Ferguson
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

#include <robot_calibration/cost_functions/magnetometer_error.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

class MagnetometerCapture
{
public:
  MagnetometerCapture(const std::string& topic, rclcpp::Node::SharedPtr node)
  {
    RCLCPP_INFO(node->get_logger(), "Subscribing to %s", topic.c_str());
    subscriber_ = node->create_subscription<sensor_msgs::msg::MagneticField>(
      topic, 1, std::bind(&MagnetometerCapture::callback, this, std::placeholders::_1)
    );
  }

  void getData(std::vector<sensor_msgs::msg::MagneticField>& data)
  {
    data = data_;
  }

private:
  void callback(sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
  {
    data_.push_back(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subscriber_;
  std::vector<sensor_msgs::msg::MagneticField> data_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("magnetometer_calibration");

  // Need to process messages in background
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  /*
   * "hard iron" offset is the result of nearby permanent ferromagnetic
   * elements (for instance, components on the PCB.
   *
   * "soft iron" effect is induced by the geomagnetic field onto normally
   * unmagnetized ferromagnetic elements.
   */
  bool soft_iron = node->declare_parameter<bool>("soft_iron", false);
  if (soft_iron)
  {
    RCLCPP_ERROR(node->get_logger(), "Soft Iron Calibration Is Not Yet Available.");
  }

  /*
   * Speed and duration to rotate the robot.
   * Setting either of these to 0.0 will skip rotation.
   */
  double rotation_velocity = node->declare_parameter<double>("rotation_velocity", 0.2);  // rad/s
  double rotation_duration = node->declare_parameter<double>("rotation_duration", 60.0);  // seconds

  /*
   * If this is a handheld IMU, you can manually
   * rotate the sensor and press ENTER when done.
   */
  bool rotation_manual = node->declare_parameter<bool>("rotation_manual", false);

  // Name of bagfile to create and/or use
  std::string bag_file_name = node->declare_parameter<std::string>(
    "bag_file_name", "/tmp/magnetometer_calibration.bag");

  // IMU topic name
  std::string mag_topic_name = "/imu/mag";

  // Get calibration data
  std::vector<sensor_msgs::msg::MagneticField> data;
  if (rotation_velocity <= 0.0 && rotation_duration <= 0.0 && !rotation_manual)
  {
    // Load calibration data from bag file
    rosbag2_cpp::Reader reader;
    reader.open(bag_file_name);
    while (reader.has_next())
    {
      try
      {
        auto bag_message = reader.read_next();
        sensor_msgs::msg::MagneticField msg;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::MagneticField> serialization;
        serialization.deserialize_message(&extracted_serialized_msg, &msg);
        data.push_back(msg);
      }
      catch (std::runtime_error&)
      {
        RCLCPP_WARN(node->get_logger(), "Unable to deserialize message");
      }
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "Loaded bag file with " << data.size() << " samples");
  }
  else
  {
    // Create calibration (and bag file)
    MagnetometerCapture capture(mag_topic_name, node);

    if (rotation_manual)
    {
      // Wait for keypress
      RCLCPP_INFO(node->get_logger(), "Rotate the sensor to many orientations.");
      RCLCPP_INFO(node->get_logger(), "Press key when done rotating");
      std::string throwaway;
      std::getline(std::cin, throwaway);
    }
    else
    {
      // Rotate the robot
      RCLCPP_INFO(node->get_logger(), "Publishing to cmd_vel");
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub =
        node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

      geometry_msgs::msg::Twist msg;
      msg.angular.z = rotation_velocity;

      RCLCPP_INFO(node->get_logger(), "Rotating robot for %f seconds.", rotation_duration);

      rclcpp::Time start = node->now();
      while (rclcpp::ok() && (node->now() - start).seconds() < rotation_duration)
      {
        pub->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }

      msg.angular.z = 0.0;
      pub->publish(msg);

      RCLCPP_INFO(node->get_logger(), "Done rotating robot.");
    }

    // Get data
    capture.getData(data);

    // Save data to bag file
    RCLCPP_INFO_STREAM(node->get_logger(), "Saving bag file with " << data.size() << " samples.");
    rosbag2_cpp::Writer writer;
    writer.open(bag_file_name);
    {
      // Create topic metadata
      rosbag2_storage::TopicMetadata metadata;
      metadata.name = mag_topic_name;
      metadata.type = "sensor_msgs/msg/MagneticField";
      metadata.serialization_format = "cdr";
      writer.create_topic(metadata);
    }
    for (auto msg : data)
    {
      // Serialize data
      rclcpp::Serialization<sensor_msgs::msg::MagneticField> serialization;
      rclcpp::SerializedMessage serialized_msg;
      serialization.serialize_message(&msg, &serialized_msg);

      // Store to bagfile
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->topic_name = mag_topic_name;
      bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        &serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
      writer.write(bag_message);
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

    RCLCPP_INFO_STREAM(node->get_logger(), "Initial estimate for hard iron offsets: [" <<
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
  RCLCPP_INFO_STREAM(node->get_logger(), summary.BriefReport());

  // Save results
  RCLCPP_INFO_STREAM(node->get_logger(), "Estimated total magnetic field: " << free_params[0] << "T");
  RCLCPP_INFO(node->get_logger(), "You can compare to expected values from");
  RCLCPP_INFO(node->get_logger(), "  https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm");

  std::cout << "mag_bias_x: " << free_params[1] << std::endl;
  std::cout << "mag_bias_y: " << free_params[2] << std::endl;
  std::cout << "mag_bias_z: " << free_params[3] << std::endl;
  delete[] free_params;

  rclcpp::shutdown();
  executor_thread.join();

  return 0;
}
