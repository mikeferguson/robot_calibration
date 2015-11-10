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

#include <math.h>
#include <robot_calibration/capture/gripper_color_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>

namespace robot_calibration
{

GripperColorFinder::GripperColorFinder(ros::NodeHandle& nh) :
  FeatureFinder(nh),
  waiting_(false)
{

  // Setup the action client
  std::string topic_name;
  nh.param<std::string>("action", topic_name, "/gripper_led_action");
  client_.reset(new LedClient(topic_name, true));
  ROS_INFO("Waiting for %s...", topic_name.c_str());
  client_->waitForServer();

  // Setup subscriber
  //nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe("/head_camera/rgb/image_raw",
      1,
      &GripperColorFinder::cameraCallback,
      this);

  // Publish where LEDs were seen
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>("led_points", 10);

  // Parameters for detection
  nh.param<double>("threshold", threshold_, 1000.0);
  nh.param<int>("max_iterations", max_iterations_, 50);

  // Parameters for LEDs themselves
  std::string gripper_led_frame;
  nh.param<std::string>("gripper_led_frame", gripper_led_frame, "wrist_roll_link");
  XmlRpc::XmlRpcValue led_poses;
  nh.getParam("poses", led_poses);
  ROS_ASSERT(led_poses.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // Each LED has a code, and pose in the gripper_led_frame
  for (int i = 0; i < led_poses.size(); ++i)
  {
    codes_.push_back(static_cast<int>(led_poses[i]["code"]));
    codes_.push_back(0);  // assumes "0" is code for "OFF"

    double x, y, z;
    x = static_cast<double>(led_poses[i]["x"]);
    y = static_cast<double>(led_poses[i]["y"]);
    z = static_cast<double>(led_poses[i]["z"]);
    trackers_.push_back(CloudDifferenceTracker(gripper_led_frame, x, y, z));

    // Publisher
    boost::shared_ptr<ros::Publisher> pub(new ros::Publisher);
    *pub = nh.advertise<sensor_msgs::Image>(static_cast<std::string>(led_poses[i]["topic"]), 10);
    tracker_publishers_.push_back(pub);
  }

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }
}


void GripperColorFinder::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  // Lock mutex before updating K
  boost::unique_lock<boost::mutex> lock(mutex_K_);

  float focal_pixels_ = msg->P[0];
  float center_x_ = msg->P[2];
  float center_y_ = msg->P[6];

  if (msg->binning_x == msg->binning_y)
  {
    if (msg->binning_x > 0)
    {
      K_ = (cv::Mat_<double>(3, 3) <<
          focal_pixels_/msg->binning_x, 0.0, center_x_/msg->binning_x,
          0.0, focal_pixels_/msg->binning_x, center_y_/msg->binning_x,
          0.0, 0.0, 1.0);
    }
    else
    {
      K_ = (cv::Mat_<double>(3, 3) <<
          focal_pixels_, 0.0, center_x_,
          0.0, focal_pixels_, center_y_,
          0.0, 0.0, 1.0);
    }
  }
  else
  {
    ROS_ERROR("binning_x is not equal to binning_y");
  }
}


void GripperColorFinder::cameraCallback(const sensor_msgs::ImageConstPtr& image)
{
  if (waiting_)
  {
    image_ = *image;
    //std::cout << image_ << std::endl;
    waiting_ = false;
  }
}

bool GripperColorFinder::waitForCloud()
{
  // Initial wait cycle so that camera is definitely up to date.
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


bool GripperColorFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  uint8_t code_idx = -1;

  std::vector<geometry_msgs::PointStamped> rgbd;
  std::vector<geometry_msgs::PointStamped> world;

  /* cv_bridge::CvImagePtr cv_ptr;
     try
     {
     cv_ptr = cv_bridge::toCvCopy(image_, "BGR8");
     }
     catch (cv_bridge::Exception& e)
     {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return false;
     } */
  robot_calibration_msgs::GripperLedCommandGoal command;
  command.led_code = 0;
  client_->sendGoal(command);
  client_->waitForResult(ros::Duration(10.0)); 

  sensor_msgs::Image prev_image;

  if (!waitForCloud())
  {
    return false;
  }
  prev_image = image_;

  // Initialize difference trackers
  for (size_t i = 0; i < trackers_.size(); ++i)
  {
    //std::cout << "***********" << image_.height << "***" << image_.width<< std::endl;
    trackers_[i].reset(image_.height, image_.width);
    //  trackers_[i].reset(cv_ptr->image.rows, cv_ptr->image.cols);
  }

  int cycles = 0;
  while (true)
  {
    // Toggle LED to next state
    code_idx = (code_idx + 1) % 8;
    command.led_code = codes_[code_idx];
    client_->sendGoal(command);
    client_->waitForResult(ros::Duration(10.0));

    if(!waitForCloud())
    {
      return false;
    }

    // Commands are organized as On-Off for each led.
    int tracker = code_idx/2;
    // Even indexes are turning on, Odd are turning off
    double weight = (code_idx%2 == 0) ? 1: -1;

    // Has each point converged?
    bool done = true;
    for (size_t t = 0; t < trackers_.size(); ++t)
    {
      done &= trackers_[t].isFound(image_, threshold_);
    }
    // We want to break only if the LED is off, so that pixel is not washed out
    if (done && (weight == -1))
    {
      break;
    }

    geometry_msgs::PointStamped led;
    led.point = trackers_[tracker].point_;
    led.header.frame_id = "wrist_roll_link";//trackers_[tracker].frame_;

    //d::cout << "tracker " << tracker << std::endl;
    trackers_[tracker].process(image_, prev_image, max_error_, weight);

    if (++cycles > max_iterations_)
    {
      ROS_ERROR("Failed to find features before using maximum iterations.");
      return false;
    }

    prev_image = image_;
  }
    // Export results
    msg->observations.resize(2);
    msg->observations[0].sensor_name = "camerargb";//camera_sensor_name_;
    msg->observations[1].sensor_name = "arm";//chain_sensor_name_;

    for (size_t t = 0; t < trackers_.size(); ++t)
    {
      geometry_msgs::PointStamped rgbd_pt;
      geometry_msgs::PointStamped world_pt;
      geometry_msgs::PointStamped world_point;

      // rgbd_pt.x  = trackers_[t];
      if (!trackers_[t].getRefinedCentroid(image_, rgbd_pt))
      {
        ROS_ERROR_STREAM("No centroid for feature " << t);
        return false;
      }

      msg->observations[0].features.push_back(rgbd_pt);
      msg->observations[0].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
  //    std::cout << "point" << rgbd_pt.point.x << "\t" << rgbd_pt.point.y << "\t" << rgbd_pt.point.z << std::endl;
      world_point.point = trackers_[t].point_;
      world_point.header.frame_id = "/wrist_roll_link";
      //std::cout <<world_point.point.x << std::endl;

      tf::TransformListener listener;
      try
      {
        listener.waitForTransform("/wrist_roll_link","/head_camera_rgb_optical_frame" , ros::Time(0), ros::Duration(3.0)); //std::cout << "done" << std::endl;
        listener.transformPoint("/head_camera_rgb_optical_frame", ros::Time(0), world_point , "/wrist_roll_link", world_pt);
        //std::cout<< "done2" << std::endl;
      }
      catch(const tf::TransformException &ex)
      {
        ROS_ERROR_STREAM("Failed to transform feature to " << trackers_[t].frame_);
        return false;
      }
      //std::cout << world_pt.point.x << "\t" << world_pt.point.y << "\t" << world_pt.point.z << "\t" << std::endl;
      double u = 574.052 * world_pt.point.x/world_pt.point.z + 319.5;
      double v = 574.052 * world_pt.point.y/world_pt.point.z + 239.5;
     
    //  std::cout << "gripper" <<std::endl;
    //  std::cout << "u" << "\t" << u << "\t" << "v" <<"\t" << v << std::endl; 
      //world_pt.point.x = u;
      //world_pt.point.y = v;
      //world_pt.point.z = 0;  
      msg->observations[1].features.push_back(world_pt);
      msg->observations[1].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
      
    } 
  

return true;
}

GripperColorFinder::CloudDifferenceTracker::CloudDifferenceTracker(
    std::string frame, double x, double y, double z) :
  frame_(frame)
{
  point_.x = x;
  point_.y = y;
  point_.z = z;
}


void GripperColorFinder::CloudDifferenceTracker::reset(size_t height, size_t width)
{
  // Save for creating images
  height_ = height;
  width_ = width;

  // Number of clouds processed.
  count_ = 0;
  // Maximum difference observed
  max_ = -1000.0;
  // Pixel this was observed in
  max_idx_ = -1;
  max_idy_ = -1;
  //std::cout << height << "\t" << width <<"******"<<std::endl;
  // Setup difference tracker
  diff_.resize(height * width);
  //std::cout << diff_.size() << "in reset" << std::endl;
  for (std::vector<double>::iterator it = diff_.begin(); it != diff_.end(); ++it)
  {
    *it = 0.0;
  }
}

bool GripperColorFinder::CloudDifferenceTracker::process(
    sensor_msgs::Image& image,
    sensor_msgs::Image& prev,
    //geometry_msgs::Point& led_point,
    double max_distance,
    double weight)
{
  if ((image.width * image.height) != diff_.size())
  {
    ROS_ERROR("Cloud size has changed");
    //std::cout << diff_.size()<< "\t" << image.width << "\t" << image.height <<std::endl;
    return false;
  }
  //std::cout <<"wth" << std::endl;

  // We want to compare each point to the expected LED pose,
  // but when the LED is on, the points will be NAN,
  // fall back on most recent distance for these points

  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImagePtr cv_ptr_prev;
  try
  { 
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv_ptr_prev = cv_bridge::toCvCopy(prev, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s " , e.what());
  }

  std::vector<cv::Mat> color;
  cv::split(cv_ptr->image , color);
  //std::cout << color.size() << std::endl;
  //std::cout << color[0].size() << std::endl;
  std::vector<cv::Mat> prev_color;
  cv::split(cv_ptr_prev->image , prev_color);

  //double last_distance = 1000.0;

  // Update each point in the tracker
  //const size_t num_points = cloud.data.size() / cloud.point_step;
  int valid = 0;
  int used = 0;
  for (size_t i = 0; i < (image.height*image.width); i++)
  {
    //    for(size_t j = 0; j< image.width;j++)
    //  {
    // If within range of LED pose... do this later
    //
    double m = i/image.width;
    double n = i%image.width;
    //std::cout << color[0].at<float>(m,n) << "\t" << color[1].at<float>(m,n) << "\t" << color[2].at<float>(m,n) << std::endl;

    //double m = i/image.width;
    //double n = i%image.height;
    double b = (double)(color[0].at<float>(m,n)) - (double)(prev_color[0].at<float>(m,n));
    double g = (double)(color[1].at<float>(m,n)) - (double)(prev_color[1].at<float>(m,n));
    double r = (double)(color[2].at<float>(m,n)) - (double)(prev_color[2].at<float>(m,n));
/*
    if (r> 0)
    {
      std::cout << "red" << std::endl;
    }

    if(b>0)
    {
      std::cout << "blue" << std::endl;
    }

    if(g>0)
    {
      std::cout<< "green" <<std::endl;
    }
*/
    if (r > 0 && g > 0 && b > 0 && weight > 0)
    {// if (r>0 && weight>0)
    
      diff_[i] += (r+b+g ) * weight;
      used++;
    }
    else if (r < 0 && g < 0 && b < 0 && weight < 0)
    {
      diff_[i] += (r+b+g ) * weight;
      used++;
    }

    // Is this a new max value?
    if (diff_[i] > max_)
    {
      max_ = diff_[i];//[j];
      max_idx_ = i;// * image.width +j;
      //max_idy_ = j;
      //std::cout << max_ << std::endl;
    }
  }
  // }
  return true;


}

bool GripperColorFinder::CloudDifferenceTracker::getRefinedCentroid(
    const sensor_msgs::Image& image,
    geometry_msgs::PointStamped& centroid)
{
  // Get initial centroid
  centroid.header = image.header;
  centroid.point.x =  max_idx_ / image.width;
  centroid.point.y =  max_idx_ % image.width;
  std::cout << "centroid" << std::endl;
  std::cout << centroid.point.x << std::endl;
  std::cout << centroid.point.y << std::endl;
  // Get a better centroid
  int points = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  //  double sum_z = 0.0;
  for (size_t i = 0; i < (image.height*image.width); i++)
  {
    // for(size_t j = 0; j < image.width; j++)
    // {

    // Using highly likely points
    if (diff_[i] > (max_*0.75))
    {
      double m = i/image.width;
      double n = i%image.width;
     // std::cout << "m" << m << "n" << n << std::endl;
      double dx = m - centroid.point.x;
      double dy = n - centroid.point.y;

      // That are less than 1cm--- verify from the max point 
      if ((dx*dx) + (dy*dy) < (10))
      {
 //       std::cout << "m" << m << "n" << n << std::endl;

        sum_x += m;//max_idx_;
        sum_y += n;//max_idy_;
        ++points;
      }
    }
  }
  //}

  if (points > 0)
  {
    centroid.point.x = (centroid.point.x + sum_x)/(points+1);
    centroid.point.y = (centroid.point.y + sum_y)/(points+1);
    centroid.point.z = 0;
  }

  return true;
}

bool GripperColorFinder::CloudDifferenceTracker::isFound(
    const sensor_msgs::Image& image,
    double threshold)
{
  // Returns true only if the max exceeds threshold
  if (max_ < threshold)
  {
  //  std::cout << "max" << max_ << std::endl;
    return false;
  }

  return true;
}
} 
