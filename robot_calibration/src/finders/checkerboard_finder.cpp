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

#include <pluginlib/class_list_macros.h>
#include <robot_calibration/capture/checkerboard_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::CheckerboardFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{
// We use a number of PC2 iterators, define the indexes here
const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

const std::string CheckerboardFinder::ChessBoard = "chess_board";
const std::string CheckerboardFinder::CircleBoardSymmetric = "circle_board_symmetric";
const std::string CheckerboardFinder::CircleBoardAsymmetric = "circle_board_asymmetric";

CheckerboardFinder::CheckerboardFinder() : waiting_(false), trials_(50U)
{
}

bool CheckerboardFinder::init(const std::string& name, ros::NodeHandle& nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  std::cerr << "namespace: " << nh.getNamespace() << std::endl;

  // Setup Scriber
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name, 1, &CheckerboardFinder::cameraCallback, this);

  // Size of checkerboard
  nh.param<int>("points_x", points_x_, 5);
  nh.param<int>("points_y", points_y_, 4);
  nh.param<double>("size", square_size_, 0.0245);

  nh.param<int32_t>("max_trials_count", trials_, 50);

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Name of checkerboard frame that will be used during optimization
  nh.param<std::string>("frame_id", frame_id_, "checkerboard");

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

  nh.param<std::string>("checkerboard_type", checkerboard_type_, ChessBoard);

  // Publish where checkerboard points were seen
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  ROS_INFO_STREAM("points_x: " << points_x_ << "\npoints_y: " << points_y_ << "\nsize: " << square_size_
                               << "\nframe_id: " << frame_id_ << "\ncamera_sensor_name_: " << camera_sensor_name_
                               << "\nchain_sensor_name: " << chain_sensor_name_);

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(nh))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

void CheckerboardFinder::cameraCallback(const sensor_msgs::PointCloud2& cloud)
{
  if (waiting_)
  {
    cloud_ = cloud;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout
bool CheckerboardFinder::waitForCloud()
{
  // Initial wait cycle so that camera is definitely up to date.
  ros::Duration(1 / 10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count && ros::ok())
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

bool CheckerboardFinder::find(robot_calibration_msgs::CalibrationData* msg)
{
  for (int32_t i = 0; (i < trials_) && (ros::ok()); ++i)
  {
    // temporary copy of msg, so we throw away all changes if findInternal() returns false
    robot_calibration_msgs::CalibrationData tmp_msg(*msg);
    if (findInternal(&tmp_msg))
    {
      *msg = tmp_msg;
      return true;
    }
  }
  return false;
}

bool CheckerboardFinder::findInternal(robot_calibration_msgs::CalibrationData* msg)
{
  geometry_msgs::PointStamped rgbd;

  // Get cloud
  if (!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  cv::Mat_<cv::Vec3b> rgb_image = getImageFromCloud();

  if (rgb_image.empty())
  {
    ROS_ERROR("CheckerboardFinder: image from cloud is empty");
    return false;
  }

  const bool is_chessboard = (checkerboard_type_ == ChessBoard);

  const bool is_circleboard =
      ((checkerboard_type_ == CircleBoardSymmetric) || (checkerboard_type_ == CircleBoardAsymmetric));

  std::vector<cv::Point2f> checker_board_image_positions;
  bool found = false;
  if (is_chessboard)
  {
    found = detectChessBoard(rgb_image, checker_board_image_positions);
  }
  else if (is_circleboard)
  {
    found = detectCircleBoard(rgb_image, checker_board_image_positions, checkerboard_type_ == CircleBoardAsymmetric);
  }
  else
  {
    ROS_ERROR_STREAM("CheckerBoardFinder: not supported checkerboard type: " << checkerboard_type_);
    return false;
  }

  if (found)
  {
    // Create PointCloud2 to publish
    sensor_msgs::PointCloud2 cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = cloud_.header.frame_id;
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(points_x_ * points_y_);
    sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

    // Set msg size
    int idx_cam = msg->observations.size() + 0;
    int idx_chain = msg->observations.size() + 1;
    msg->observations.resize(msg->observations.size() + 2);
    msg->observations[idx_cam].sensor_name = camera_sensor_name_;
    msg->observations[idx_chain].sensor_name = chain_sensor_name_;

    msg->observations[idx_cam].features.resize(points_x_ * points_y_);

    if (is_circleboard)
    {
      msg->observations[idx_chain].features = computeObjectPointsChessBoard();
    }
    else
    {
      msg->observations[idx_chain].features =
          computeObjectPointsCircleBoard(checkerboard_type_ == CircleBoardAsymmetric);
    }

    if (msg->observations[idx_chain].features.size() != checker_board_image_positions.size())
    {
      ROS_ERROR_STREAM("CheckerBoardDetector:: Rejecting observation because size of image points: "
                       << checker_board_image_positions.size()
                       << ", is not equal to size of object points: " << msg->observations[idx_chain].features.size());
      return false;
    }

    // Fill in the headers
    rgbd.header = cloud_.header;

    // Fill in message
    sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud_, "x");
    for (size_t i = 0; i < checker_board_image_positions.size(); ++i)
    {
      // Get 3d point
      size_t index = static_cast<size_t>(checker_board_image_positions[i].y) * cloud_.width +
                     static_cast<size_t>(checker_board_image_positions[i].x);
      rgbd.point.x = (xyz + index)[X];
      rgbd.point.y = (xyz + index)[Y];
      rgbd.point.z = (xyz + index)[Z];

      // Do not accept NANs
      if (std::isnan(rgbd.point.x) || std::isnan(rgbd.point.y) || std::isnan(rgbd.point.z))
      {
        ROS_ERROR_STREAM("CheckerBoardDetector:: Rejecting observation due to NAN point on " << i);
        return false;
      }

      // Do not accept (0.0, 0.0, 0.0)
      const double eps = static_cast<double>(10.0) * std::numeric_limits<double>::epsilon();
      if ((std::fabs(rgbd.point.x) < eps) || (std::fabs(rgbd.point.y) < eps) || (std::fabs(rgbd.point.z) < eps))
      {
        ROS_ERROR_STREAM("CheckerBoardDetector:: Rejecting observation due to (0.0, 0.0, 0.0) point on " << i);
        return false;
      }

      msg->observations[idx_cam].features[i] = rgbd;
      msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();

      // Visualize
      iter_cloud[0] = rgbd.point.x;
      iter_cloud[1] = rgbd.point.y;
      iter_cloud[2] = rgbd.point.z;
      ++iter_cloud;
    }

    // Add debug cloud to message
    if (output_debug_)
    {
      msg->observations[idx_cam].cloud = cloud_;
    }

    // Publish results
    publisher_.publish(cloud);

    // Found all points
    return true;
  }

  return false;
}

bool CheckerboardFinder::detectChessBoard(const cv::Mat_<cv::Vec3b>& image, std::vector<cv::Point2f>& points) const
{
  ROS_INFO_STREAM("CheckerboardFinder: detecting chessboard corners: points_y: " << points_y_
                                                                                 << ", points_x: " << points_x_);
  const bool found =
      cv::findChessboardCorners(image, cv::Size(points_x_, points_y_), points, CV_CALIB_CB_ADAPTIVE_THRESH);

  if (found)
  {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(gray, points, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }

  return found;
}

bool CheckerboardFinder::detectCircleBoard(const cv::Mat_<cv::Vec3b>& image, std::vector<cv::Point2f>& points,
                                           const bool asymmetric) const
{
  ROS_INFO_STREAM("CheckerboardFinder: detecting circles: points_y: " << points_y_ << ", points_x: " << points_x_
                                                                      << ", asymmetric: " << asymmetric);
  return cv::findCirclesGrid(image, cv::Size(points_x_, points_y_), points,
                             (asymmetric) ? cv::CALIB_CB_ASYMMETRIC_GRID : cv::CALIB_CB_SYMMETRIC_GRID);
}

cv::Mat_<cv::Vec3b> CheckerboardFinder::getImageFromCloud() const
{
  if ((cloud_.height + cloud_.width) <= 1U)
  {
    return cv::Mat_<cv::Vec3b>();
  }

  cv::Mat_<cv::Vec3b> image(cloud_.height, cloud_.width);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> rgb(cloud_, "rgb");
  for (size_t y = 0; y < cloud_.height; y++)
  {
    for (size_t x = 0; x < cloud_.width; x++)
    {
      image(y, x)[0U] = rgb[0U];
      image(y, x)[1U] = rgb[1U];
      image(y, x)[2U] = rgb[2U];
      ++rgb;
    }
  }
  return image;
}

std::vector<geometry_msgs::PointStamped> CheckerboardFinder::computeObjectPointsCircleBoard(const bool asymmetric) const
{
  if (asymmetric)
  {
    cv::Size pattern_size(points_x_, points_y_);
    std::vector<geometry_msgs::PointStamped> object_points;
    for (int i = 0; i < pattern_size.height; i++)
    {
      for (int j = 0; j < pattern_size.width; j++)
      {
        geometry_msgs::PointStamped p;
        p.header.frame_id = frame_id_;
        p.header.stamp = cloud_.header.stamp;
        p.point.x = static_cast<double>((2 * j + i % 2) * square_size_);
        p.point.y = static_cast<double>(i * square_size_);
        p.point.z = 0.0;

        object_points.push_back(p);
      }
    }
    return object_points;
  }
  else
  {
    return computeObjectPointsChessBoard();
  }
}

std::vector<geometry_msgs::PointStamped> CheckerboardFinder::computeObjectPointsChessBoard() const
{
  std::vector<geometry_msgs::PointStamped> object_points;

  for (size_t i = 0; i < static_cast<size_t>(points_x_ * points_y_); ++i)
  {
    geometry_msgs::PointStamped p;
    p.header.frame_id = frame_id_;
    p.header.stamp = cloud_.header.stamp;
    p.point.x = (i % points_x_) * square_size_;
    p.point.y = (i / points_x_) * square_size_;
    p.point.z = 0.0;

    object_points.push_back(p);
  }

  return object_points;
}

}  // namespace robot_calibration
