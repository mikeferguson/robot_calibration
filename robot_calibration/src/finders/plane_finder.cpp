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

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pluginlib/class_list_macros.h>
#include <robot_calibration/capture/plane_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::PlaneFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

PlaneFinder::PlaneFinder() :
  tfListener_(tfBuffer_), waiting_(false)
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
        tfBuffer_.transform(p, p_out, transform_frame_);
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

  // Find centroid
  Eigen::Vector3d centroid(points.row(0).mean(), points.row(1).mean(), points.row(2).mean());

  // Center the cloud
  points.row(0).array() -= centroid(0);
  points.row(1).array() -= centroid(1);
  points.row(2).array() -= centroid(2);

  // Find the plane
  auto svd = points.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector3d normal = svd.matrixU().rightCols<1>();

  // Get the rest of plane equation
  double d = -(normal(0) * centroid(0) + normal(1) * centroid(1) + normal(2) * centroid(2));

  ROS_INFO("Found plane with parameters: %f %f %f %f", normal(0), normal(1), normal(2), d);

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
    double dist = normal(0) * (xyz + i)[X] + normal(1) * (xyz + i)[Y] + normal(2) * (xyz + i)[Z] + d;

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
  // Determine number of points to output
  size_t points_total = std::min(points_max_, static_cast<int>(cloud.width));
  ROS_INFO_STREAM("Got " << cloud.width << " points from plane, using " << points_total);

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

  // Fill in observation
  size_t step = cloud.width / points_total;
  size_t index = step / 2;
  sensor_msgs::PointCloud2Iterator<float> viz_cloud_iter(viz_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud, "x");
  for (size_t i = 0; index < cloud.width && i < points_total; ++i)
  {
    // Get (untransformed) 3d point
    geometry_msgs::PointStamped rgbd;
    rgbd.point.x = (xyz + index)[X];
    rgbd.point.y = (xyz + index)[Y];
    rgbd.point.z = (xyz + index)[Z];

    // Add to observation
    msg->observations[idx_cam].features.push_back(rgbd);

    // Copy to cloud for publishing
    viz_cloud_iter[0] = rgbd.point.x;
    viz_cloud_iter[1] = rgbd.point.y;
    viz_cloud_iter[2] = rgbd.point.z;
    ++viz_cloud_iter;

    // Next point
    index += step;
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
