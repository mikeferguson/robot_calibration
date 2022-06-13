/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2015-2017 Fetch Robotics Inc.
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

// Author: Niharika Arora, Michael Ferguson

#include <math.h>
#include <stdlib.h>

#include <robot_calibration/finders/plane_finder.hpp>
#include <robot_calibration/util/eigen_geometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("plane_finder");

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

// Helper function to sample points from a cloud
int sampleCloud(const sensor_msgs::msg::PointCloud2& points,
                double sample_distance, size_t max_points,
                std::vector<geometry_msgs::msg::PointStamped>& sampled_points)
{
  // Square distance for efficiency
  double max_dist_sq = sample_distance * sample_distance;

  // Iterate through cloud
  sensor_msgs::PointCloud2ConstIterator<float> points_iter(points, "x");
  for (size_t i = 0; i < points.width; ++i)
  {
    // Get (untransformed) 3d point
    geometry_msgs::msg::PointStamped rgbd;
    rgbd.point.x = (points_iter + i)[X];
    rgbd.point.y = (points_iter + i)[Y];
    rgbd.point.z = (points_iter + i)[Z];

    // Is this far enough from our current points?
    bool include_point = true;
    for (auto point : sampled_points)
    {
      double dx = point.point.x - rgbd.point.x;
      double dy = point.point.y - rgbd.point.y;
      double dz = point.point.z - rgbd.point.z;

      double dist_sq = dx * dx + dy * dy + dz * dz;
      if (dist_sq < max_dist_sq)
      {
        include_point = false;
        break;
      }
    }

    if (include_point)
    {
      // Add this to samples
      sampled_points.push_back(rgbd);
    }

    if (sampled_points.size() >= max_points)
    {
      // Done sampling
      break;
    }
  }

  RCLCPP_INFO(LOGGER, "Extracted %lu points with sampling distance of %f", sampled_points.size(), sample_distance);

  return sampled_points.size();
}

PlaneFinder::PlaneFinder() :
  waiting_(false)
{
}

bool PlaneFinder::init(const std::string& name,
                       std::shared_ptr<tf2_ros::Buffer> buffer,
                       rclcpp::Node::SharedPtr node)
{
  if (!FeatureFinder::init(name, buffer, node))
  {
    return false;
  }

  clock_ = node->get_clock();

  // We subscribe to a PointCloud2
  std::string topic_name =
    node->declare_parameter<std::string>(name + ".topic", name + "/points");
  subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name, 1, std::bind(&PlaneFinder::cameraCallback, this, std::placeholders::_1));

  // Name of the sensor model that will be used during optimization
  plane_sensor_name_ = node->declare_parameter<std::string>(name + ".camera_sensor_name", "camera");

  // Maximum number of valid points to include in the observation
  points_max_ = node->declare_parameter<int>(name + ".points_max", 60);

  // When downsampling cloud, start by selecting points at least this far apart
  initial_sampling_distance_ = node->declare_parameter<double>(
    name + ".initial_sample_distance", 0.2);

  // Maximum distance from plane that point can be located
  plane_tolerance_ = node->declare_parameter<double>(name + ".tolerance", 0.02);

  // Frame to transform point cloud into before applying limits below
  //   if specified as "none", cloud will be processed in sensor frame
  transform_frame_  = node->declare_parameter<std::string>(
    name + ".transform_frame", "base_link");

  // Valid points must lie within this box, in the transform_frame
  min_x_ = node->declare_parameter<double>(name + ".min_x", -2.0);
  max_x_ = node->declare_parameter<double>(name + ".max_x", 2.0);
  min_y_ = node->declare_parameter<double>(name + ".min_y", -2.0);
  max_y_ = node->declare_parameter<double>(name + ".max_y", 2.0);
  min_z_ = node->declare_parameter<double>(name + ".min_z", -2.0);
  max_z_ = node->declare_parameter<double>(name + ".max_z", 2.0);

  // Parameters for RANSAC
  ransac_iterations_ = node->declare_parameter<int>(name + ".ransac_iterations", 100);
  ransac_points_ = node->declare_parameter<int>(name + ".ransac_points", 35);

  // Optional normal vector that found plane should align with
  // Leave all parameters set to 0 to disable check and simply take best fitting plane
  double a, b, c;
  a = node->declare_parameter<double>(name + ".normal_a", 0.0);
  b = node->declare_parameter<double>(name + ".normal_b", 0.0);
  c = node->declare_parameter<double>(name + ".normal_c", 0.0);
  desired_normal_ = Eigen::Vector3d(a, b, c);
  // If normal vector is defined, the plane normal must be aligned within this angle (in radians)
  cos_normal_angle_ = node->declare_parameter<double>(name + ".normal_angle", 0.349065);  // 20 degrees
  cos_normal_angle_ = cos(cos_normal_angle_);

  // Should we include debug image/cloud in observations
  output_debug_ = node->declare_parameter<bool>(name + ".debug", false);

  // Publish the observation as a PointCloud2
  publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(name + "_points", 10);

  // Make sure we have CameraInfo before starting
  if (!depth_camera_manager_.init(name, node, LOGGER))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

void PlaneFinder::cameraCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (waiting_)
  {
    cloud_ = *cloud;
    waiting_ = false;
  }
}

bool PlaneFinder::waitForCloud()
{
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    // TODO ros::spinOnce();
  }
  RCLCPP_ERROR(LOGGER, "Failed to get cloud");
  return !waiting_;
}

bool PlaneFinder::find(robot_calibration_msgs::msg::CalibrationData * msg)
{
  if (!waitForCloud())
  {
    RCLCPP_ERROR(LOGGER, "No point cloud data");
    return false;
  }

  removeInvalidPoints(cloud_, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
  sensor_msgs::msg::PointCloud2 plane = extractPlane(cloud_);
  extractObservation(plane_sensor_name_, plane, msg, publisher_);

  // Report success
  return true;
}

void PlaneFinder::removeInvalidPoints(sensor_msgs::msg::PointCloud2& cloud,
  double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
  //  Remove any point that is invalid or not with our tolerance
  size_t num_points = cloud.width * cloud.height;
  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");

  bool do_transform = transform_frame_ != "none";  // This can go away once the cloud gets transformed outside loop
  size_t j = 0;
  for (size_t i = 0; i < num_points; i++)
  {
    geometry_msgs::msg::PointStamped p;
    p.point.x = (xyz + i)[X];
    p.point.y = (xyz + i)[Y];
    p.point.z = (xyz + i)[Z];

    // Remove the NaNs in the point cloud
    if (!std::isfinite(p.point.x) || !std::isfinite(p.point.y) || !std::isfinite(p.point.z))
    {
      continue;
    }

    // Remove the points immediately in front of the camera in the point cloud
    // NOTE : This is to handle sensors that publish zeros instead of NaNs in the point cloud
    if (p.point.z == 0)
    {
      continue;
    }

    // Get transform (if any)
    geometry_msgs::msg::PointStamped p_out;
    if (do_transform)
    {
      p.header.stamp.sec = 0;
      p.header.stamp.nanosec = 0;
      p.header.frame_id = cloud_.header.frame_id;
      try
      {
        tf2_buffer_->transform(p, p_out, transform_frame_);
      }
      catch (tf2::TransformException& ex)
      {
        RCLCPP_ERROR(LOGGER, "%s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
        continue;
      }
    }
    else
    {
      p_out = p;
    }

    // Test the transformed point
    if (p_out.point.x < min_x || p_out.point.x > max_x || p_out.point.y < min_y || p_out.point.y > max_y ||
        p_out.point.z < min_z || p_out.point.z > max_z)
    {
      continue;
    }

    // This is a valid point, move it forward
    (cloud_iter + j)[X] = (xyz + i)[X];
    (cloud_iter + j)[Y] = (xyz + i)[Y];
    (cloud_iter + j)[Z] = (xyz + i)[Z];
    j++;
  }
  cloud.height = 1;
  cloud.width  = j;
  cloud.data.resize(cloud.width * cloud.point_step);
}

sensor_msgs::msg::PointCloud2 PlaneFinder::extractPlane(sensor_msgs::msg::PointCloud2& cloud)
{
  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud, "x");

  // Copy cloud to eigen matrix for SVD
  Eigen::MatrixXd points(3, cloud.width);
  for (size_t i = 0; i < cloud.width; ++i)
  {
    points(0, i) = (xyz + i)[X];
    points(1, i) = (xyz + i)[Y];
    points(2, i) = (xyz + i)[Z];
  }

  // Find the best fit plane
  Eigen::Vector3d best_normal(0, 0, 1);
  double best_d = 0.0;
  int best_fit = -1;
  for (int i = 0; i < ransac_iterations_; ++i)
  {
    // Select random points
    Eigen::MatrixXd test_points(3, ransac_points_);
    for (int p = 0; p < ransac_points_; ++p)
    {
      int index = rand() % cloud.width;
      test_points(0, p) = points(0, index);
      test_points(1, p) = points(1, index);
      test_points(2, p) = points(2, index);
    }

    // Get plane for this test set
    Eigen::Vector3d normal;
    double d = 0.0;
    getPlane(test_points, normal, d);

    // If we have desired normal, check if plane is well enough aligned
    if (desired_normal_.norm() > 0.1)
    {
      // We might have to transform the normal to a different frame
      geometry_msgs::msg::Vector3Stamped transformed_normal;
      transformed_normal.header = cloud.header;
      transformed_normal.vector.x = normal(0);
      transformed_normal.vector.y = normal(1);
      transformed_normal.vector.z = normal(2);

      if (transform_frame_ != "none")
      {
        try
        {
          tf2_buffer_->transform(transformed_normal, transformed_normal, transform_frame_);
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_ERROR(LOGGER, "%s", ex.what());
          continue;
        }
      }

      Eigen::Vector3d transformed(transformed_normal.vector.x,
                                  transformed_normal.vector.y,
                                  transformed_normal.vector.z);

      // a.dot(b) = norm(a) * norm(b) * cos(angle between a & b)
      double angle = transformed.dot(desired_normal_) / desired_normal_.norm() / transformed.norm();
      if (std::fabs(angle) < cos_normal_angle_)
      {
        continue;
      }
    }

    // Test how many fit
    int fit_count = 0;
    for (size_t i = 0; i < cloud.width; ++i)
    {
      // Compute distance to plane
      double dist = normal(0) * (xyz + i)[X] + normal(1) * (xyz + i)[Y] + normal(2) * (xyz + i)[Z] + d;
      if (std::fabs(dist) < plane_tolerance_)
      {
        ++fit_count;
      }
    }

    if (fit_count > best_fit)
    {
      // Accept this as our best fit
      best_fit = fit_count;
      best_normal = normal;
      best_d = d;
    }
  }
  // Note: parameters are in cloud.header.frame_id and not transform_frame
  RCLCPP_INFO(LOGGER, "Found plane with parameters: %f %f %f %f", best_normal(0), best_normal(1), best_normal(2), best_d);

  // Create a point cloud for the plane
  sensor_msgs::msg::PointCloud2 plane_cloud;
  plane_cloud.width = 0;
  plane_cloud.height = 0;
  plane_cloud.header.stamp = clock_->now();
  plane_cloud.header.frame_id = cloud.header.frame_id;
  sensor_msgs::PointCloud2Modifier plane_cloud_mod(plane_cloud);
  plane_cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  plane_cloud_mod.resize(cloud.width);
  sensor_msgs::PointCloud2Iterator<float> plane_iter(plane_cloud, "x");

  // Extract points close to plane
  sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
  size_t plane_points = 0;
  size_t cloud_points = 0;
  for (size_t i = 0; i < cloud.width; ++i)
  {
    // Compute distance to plane
    double dist = best_normal(0) * (xyz + i)[X] + best_normal(1) * (xyz + i)[Y] + best_normal(2) * (xyz + i)[Z] + best_d;

    if (std::fabs(dist) < plane_tolerance_)
    {
      // Part of the plane
      (plane_iter + plane_points)[X] = (xyz + i)[X];
      (plane_iter + plane_points)[Y] = (xyz + i)[Y];
      (plane_iter + plane_points)[Z] = (xyz + i)[Z];
      ++plane_points;
    }
    else
    {
      // Part of cloud, move it forward
      (cloud_iter + cloud_points)[X] = (xyz + i)[X];
      (cloud_iter + cloud_points)[Y] = (xyz + i)[Y];
      (cloud_iter + cloud_points)[Z] = (xyz + i)[Z];
      ++cloud_points;
    }
  }

  // Resize clouds
  cloud.height = 1;
  cloud.width  = cloud_points;
  cloud.data.resize(cloud.width * cloud.point_step);

  plane_cloud.height = 1;
  plane_cloud.width  = plane_points;
  plane_cloud.data.resize(plane_cloud.width * plane_cloud.point_step);

  RCLCPP_INFO(LOGGER, "Extracted plane with %d points", plane_cloud.width);

  return plane_cloud;
}

void PlaneFinder::extractObservation(const std::string& sensor_name,
                                     const sensor_msgs::msg::PointCloud2& cloud,
                                     robot_calibration_msgs::msg::CalibrationData * msg,
                                     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher)
{
  if (static_cast<int>(cloud.width) == 0)
  {
    RCLCPP_WARN(LOGGER, "No points in observation, skipping");
    return;
  }

  // Determine number of points to output
  size_t points_total = std::min(points_max_, static_cast<int>(cloud.width));
  RCLCPP_INFO_STREAM(LOGGER, "Got " << cloud.width << " points for observation, using " << points_total);

  // Create PointCloud2 to publish
  sensor_msgs::msg::PointCloud2 viz_cloud;
  viz_cloud.width = 0;
  viz_cloud.height = 0;
  viz_cloud.header.stamp = clock_->now();
  viz_cloud.header.frame_id = cloud.header.frame_id;
  sensor_msgs::PointCloud2Modifier cloud_mod(viz_cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(points_total);

  // Setup observation
  int idx_cam = msg->observations.size();
  msg->observations.resize(msg->observations.size() + 1);
  msg->observations[idx_cam].sensor_name = sensor_name;
  msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();

  // Get observation points
  std::vector<geometry_msgs::msg::PointStamped> sampled_points;
  double sample_distance = initial_sampling_distance_;
  while (sampled_points.size() < points_total)
  {
    sampleCloud(cloud, sample_distance, points_total, sampled_points);
    sample_distance /= 2;
  }

  // Fill in observation
  sensor_msgs::PointCloud2Iterator<float> viz_cloud_iter(viz_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud, "x");
  for (auto rgbd : sampled_points)
  {
    // Add to observation
    msg->observations[idx_cam].features.push_back(rgbd);

    // Copy to cloud for publishing
    viz_cloud_iter[0] = rgbd.point.x;
    viz_cloud_iter[1] = rgbd.point.y;
    viz_cloud_iter[2] = rgbd.point.z;
    ++viz_cloud_iter;
  }

  // Add debug cloud to message
  if (output_debug_)
  {
    msg->observations[idx_cam].cloud = cloud;
  }

  if (publisher)
  {
    // Publish debug info
    publisher->publish(viz_cloud);
  }
}

}  // namespace robot_calibration

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_calibration::PlaneFinder, robot_calibration::FeatureFinder)
