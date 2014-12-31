/*
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

#ifndef UBR_CALIBRATION_CAPTURE_LED_FINDER_H_
#define UBR_CALIBRATION_CAPTURE_LED_FINDER_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ubr_calibration/capture/feature_finder.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <ubr_calibration/CalibrationData.h>
#include <ubr_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace ubr_calibration
{

typedef actionlib::SimpleActionClient<ubr_msgs::GripperLedCommandAction> LedClient;

/**
 *  \brief This class processes the point cloud input to find the LED
 */
class LedFinder : public FeatureFinder
{
public:
  LedFinder(ros::NodeHandle & n);

  /**
   * \brief Attempts to find the led in incoming data.
   * \param msg CalibrationData instance to fill in with led point information.
   * \returns True if point has been filled in.
   */
  bool find(ubr_calibration::CalibrationData * msg);

private:
  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::Image
  ros::Publisher publisher_;  /// Outgoing sensor_msgs::PointCloud2
  LedClient client_;

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;

  double threshold_;  /// Minimum value of diffs in order to trigger that this is an LED
  int max_iterations_;  /// Maximum number of cycles before we abort finding the LED
  bool output_debug_image_;
  std::string gripper_led_frame_;

  tf::TransformListener listener_;
};

}  // namespace ubr_calibration

#endif  // UBR_CALIBRATION_CAPTURE_LED_FINDER_H_
