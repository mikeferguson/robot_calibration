/*
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

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <robot_calibration/capture/feature_finder.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

namespace robot_calibration
{

/** @brief This class processes the point cloud input to find the LED. */
class LedFinder : public FeatureFinder
{
  /** @brief Internally used within LED finder to track each of several LEDs. */
  struct CloudDifferenceTracker
  {

    CloudDifferenceTracker(std::string frame, double x, double y, double z);

    // Weight should be +/- 1 typically
    bool process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev,
                 double weight);

    // Have we found the LED?
    bool isFound(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 double threshold);

    // Gives a refined centroid using multiple points
    bool getRefinedCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            geometry_msgs::PointStamped& point);

    // Reset the tracker
    void reset(size_t size);

    std::vector<double> diff_;
    double max_;
    int max_idx_;
    int count_;
    std::string frame_;  // frame of led coordinates
    geometry_msgs::Point point;  //coordinates of led this is tracking
  };

  typedef actionlib::SimpleActionClient<robot_calibration_msgs::GripperLedCommandAction> LedClient;

public:
  LedFinder(ros::NodeHandle & n);

  /**
   * \brief Attempts to find the led in incoming data.
   * \param msg CalibrationData instance to fill in with led point information.
   * \returns True if point has been filled in.
   */
  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2
  ros::Publisher publisher_;  /// Outgoing sensor_msgs::PointCloud2
  boost::scoped_ptr<LedClient> client_;

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;

  std::vector<CloudDifferenceTracker> trackers_;
  std::vector<uint8_t> codes_;

  tf::TransformListener listener_;

  /*
   * ROS Parameters
   */
  double max_error_;    /// Maximum distance led can be from expected pose
  double max_inconsistency_;

  double threshold_;    /// Minimum value of diffs in order to trigger that this is an LED
  int max_iterations_;  /// Maximum number of cycles before we abort finding the LED

  bool output_debug_;   /// Should we output debug image/cloud?
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_LED_FINDER_H
