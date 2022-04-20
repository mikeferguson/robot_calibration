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

#include <pluginlib/class_list_macros.h>
#include <robot_calibration/capture/plane_finder.h>
#include <robot_calibration/eigen_geometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::PlaneFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

// Helper function to sample points from a cloud
int sampleCloud(const sensor_msgs::PointCloud2& points,
                double sample_distance, size_t max_points,
                std::vector<geometry_msgs::PointStamped>& sampled_points)
{
  // Square distance for efficiency
  double max_dist_sq = sample_distance * sample_distance;

  // Iterate through cloud
  sensor_msgs::PointCloud2ConstIterator<float> points_iter(points, "x");
  for (size_t i = 0; i < points.width; ++i)
  {
    // Get (untransformed) 3d point
    geometry_msgs::PointStamped rgbd;
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

  ROS_INFO("Extracted %lu points with sampling distance of %f", sampled_points.size(), sample_distance);

  return sampled_points.size();
}

PlaneFinder::PlaneFinder() :
  tf_listener_(tf_buffer_), waiting_(false)
{
}

bool PlaneFinder::init(const std::string& name,
                       ros::NodeHandle & nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  // We subscribe to a PointCloud2
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &PlaneFinder::cameraCallback,
                             this);

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("camera_sensor_name", plane_sensor_name_, "camera");

  // Maximum number of valid points to include in the observation
  nh.param<int>("points_max", points_max_, 60);

  // When downsampling cloud, start by selecting points at least this far apart
  nh.param<double>("initial_sample_distance", initial_sampling_distance_, 0.2);

  // Maximum distance from plane that point can be located
  nh.param<double>("tolerance", plane_tolerance_, 0.02);

  // Frame to transform point cloud into before applying limits below
  //   if specified as "none", cloud will be processed in sensor frame
  nh.param<std::string>("transform_frame", transform_frame_, "base_link");

  // Valid points must lie within this box, in the transform_frame
  nh.param<double>("min_x", min_x_, -2.0);
  nh.param<double>("max_x", max_x_, 2.0);
  nh.param<double>("min_y", min_y_, -2.0);
  nh.param<double>("max_y", max_y_, 2.0);
  nh.param<double>("min_z", min_z_, 0.0);
  nh.param<double>("max_z", max_z_, 2.0);

  // Parameters for RANSAC
  nh.param<int>("ransac_iterations", ransac_iterations_, 100);
  nh.param<int>("ransac_points", ransac_points_, 35);

  // Optional normal vector that found plane should align with
  // Leave all parameters set to 0 to disable check and simply take best fitting plane
  double a, b, c;
  nh.param<double>("normal_a", a, 0.0);
  nh.param<double>("normal_b", b, 0.0);
  nh.param<double>("normal_c", c, 0.0);
  desired_normal_ = Eigen::Vector3d(a, b, c);
  // If normal vector is defined, the plane normal must be aligned within this angle (in radians)
  nh.param<double>("normal_angle", cos_normal_angle_, 0.349065);  // Default is 20 degrees
  cos_normal_angle_ = cos(cos_normal_angle_);

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Publish the observation as a PointCloud2
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  // Make sure we have CameraInfo before starting
  if (!depth_camera_manager_.init(nh))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

void PlaneFinder::cameraCallback(const sensor_msgs::PointCloud2& cloud)
{
  if (waiting_)
  {
    cloud_ = cloud;
    waiting_ = false;
  }
}

bool PlaneFinder::waitForCloud()
{
  ros::Duration(1/10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  ROS_ERROR("Failed to get cloud");
  return !waiting_;
}

bool PlaneFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  if (!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  removeInvalidPoints(cloud_, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
  sensor_msgs::PointCloud2 plane = extractPlane(cloud_);
  extractObservation(plane_sensor_name_, plane, msg, &publisher_);

  // Report success
  return true;
}

void PlaneFinder::removeInvalidPoints(sensor_msgs::PointCloud2& cloud,
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
    geometry_msgs::PointStamped p;
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
    geometry_msgs::PointStamped p_out;
    if (do_transform)
    {
      p.header.stamp = ros::Time(0);
      p.header.frame_id = cloud_.header.frame_id;
      try
      {
        tf_buffer_.transform(p, p_out, transform_frame_);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
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

sensor_msgs::PointCloud2 PlaneFinder::extractPlane(sensor_msgs::PointCloud2& cloud)
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
      geometry_msgs::Vector3Stamped transformed_normal;
      transformed_normal.header = cloud.header;
      transformed_normal.vector.x = normal(0);
      transformed_normal.vector.y = normal(1);
      transformed_normal.vector.z = normal(2);

      if (transform_frame_ != "none")
      {
        try
        {
          tf_buffer_.transform(transformed_normal, transformed_normal, transform_frame_);
        }
        catch (tf2::TransformException& ex)
        {
          ROS_ERROR("%s", ex.what());
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
  ROS_INFO("Found plane with parameters: %f %f %f %f", best_normal(0), best_normal(1), best_normal(2), best_d);

  // Create a point cloud for the plane
  sensor_msgs::PointCloud2 plane_cloud;
  plane_cloud.width = 0;
  plane_cloud.height = 0;
  plane_cloud.header.stamp = ros::Time::now();
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

  ROS_INFO("Extracted plane with %d points", plane_cloud.width);

  return plane_cloud;
}

void PlaneFinder::extractObservation(const std::string& sensor_name,
                                     const sensor_msgs::PointCloud2& cloud,
                                     robot_calibration_msgs::CalibrationData * msg,
                                     ros::Publisher* publisher)
{
  if (static_cast<int>(cloud.width) == 0)
  {
    ROS_WARN("No points in observation, skipping");
    return;
  }

  // Determine number of points to output
  size_t points_total = std::min(points_max_, static_cast<int>(cloud.width));
  ROS_INFO_STREAM("Got " << cloud.width << " points for observation, using " << points_total);

  // Create PointCloud2 to publish
  sensor_msgs::PointCloud2 viz_cloud;
  viz_cloud.width = 0;
  viz_cloud.height = 0;
  viz_cloud.header.stamp = ros::Time::now();
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
  std::vector<geometry_msgs::PointStamped> sampled_points;
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
