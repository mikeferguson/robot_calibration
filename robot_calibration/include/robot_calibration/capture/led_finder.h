/*
 * Copyright (C) 2022 Michael Ferguson
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
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

#ifndef ROBOT_CALIBRATION_CAPTURE_LED_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_LED_FINDER_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/plugins/feature_finder.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>
#include <robot_calibration_msgs/action/gripper_led_command.hpp>

namespace robot_calibration
{

/** @brief This class processes the point cloud input to find the LED. */
class LedFinder : public FeatureFinder
{
  /** @brief Internally used within LED finder to track each of several LEDs. */
  struct CloudDifferenceTracker
  {
    CloudDifferenceTracker(std::string frame, double x, double y, double z);

    /**
     * @brief Update the tracker based on new cloud compared to previous.
     * @param cloud The newest cloud
     * @param prev The previous cloud
     * @param led_point The expected pose of this led in cloud frame
     * @param max_distance The maximum distance from expected led pose
     *        that we should consider changes.
     * @param weight Whether the change between frames should increase
     *        or decrease the LED point values. Should be +/- 1 typically.
     */
    bool process(sensor_msgs::msg::PointCloud2& cloud,
                 sensor_msgs::msg::PointCloud2& prev,
                 geometry_msgs::msg::Point& led_point,
                 double max_distance,
                 double weight);

    // Have we found the LED?
    bool isFound(const sensor_msgs::msg::PointCloud2& cloud,
                 double threshold);

    // Gives a refined centroid using multiple points
    bool getRefinedCentroid(const sensor_msgs::msg::PointCloud2& cloud,
                            geometry_msgs::msg::PointStamped& centroid);

    // Reset the tracker
    void reset(size_t height, size_t width);

    // Get an image of tracker status
    sensor_msgs::msg::Image getImage();

    std::vector<double> diff_;
    double max_;
    int max_idx_;
    int count_;
    size_t height_, width_;
    std::string frame_;  // frame of led coordinates
    geometry_msgs::msg::Point point_;  //coordinates of led this is tracking
  };

  using LedAction = robot_calibration_msgs::action::GripperLedCommand;

public:
  LedFinder();
  bool init(const std::string& name,
            std::shared_ptr<tf2_ros::Buffer> buffer,
            rclcpp::Node::SharedPtr node);
  bool find(robot_calibration_msgs::msg::CalibrationData * msg);

private:
  void cameraCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  bool waitForCloud();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp_action::Client<LedAction>::SharedPtr client_;
  rclcpp::Clock::SharedPtr clock_;

  bool waiting_;
  sensor_msgs::msg::PointCloud2 cloud_;

  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> tracker_publishers_;
  std::vector<CloudDifferenceTracker> trackers_;
  std::vector<uint8_t> codes_;

  DepthCameraInfoManager depth_camera_manager_;

  /*
   * ROS Parameters
   */
  double max_error_;    /// Maximum distance led can be from expected pose
  double max_inconsistency_;

  double threshold_;    /// Minimum value of diffs in order to trigger that this is an LED
  int max_iterations_;  /// Maximum number of cycles before we abort finding the LED

  bool output_debug_;   /// Should we output debug image/cloud?

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_LED_FINDER_H
