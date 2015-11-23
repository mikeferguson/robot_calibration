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

#ifndef ROBOT_CALIBRATION_CERES_GRIPPER_DEPTH_ERROR_H
#define ROBOT_CALIBRATION_CERES_GRIPPER_DEPTH_ERROR_H

#include <string>
#include <ceres/ceres.h>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <opencv2/rgbd/rgbd.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace robot_calibration
{

struct GripperDepthError
{
  GripperDepthError(Camera3dModel* camera_model,
                   ChainModel* arm_model,
                   CalibrationOffsetParser* offsets,
                   robot_calibration_msgs::CalibrationData& data)
  {
    camera_model_ = camera_model;
    arm_model_ = arm_model;
    offsets_ = offsets;
    data_ = data;
  }

  virtual ~GripperDepthError() {}

  bool operator()(double const * const * free_params,
                  double* residuals) const
  {
    // Update calibration offsets based on free params
    offsets_->update(free_params[0]);

//    std::cout <<  data_.observations[0].features[0].point.x <<std::endl;
//    std::cout <<  data_.observations[1].features.size() <<std::endl;
//    std::cout <<  data_.observations[0].sensor_name <<std::endl;
//    std::cout <<  data_.observations[1].sensor_name <<std::endl;

    // Project the camera observations
    std::vector<geometry_msgs::PointStamped> camera_pts =
        camera_model_->project(data_, *offsets_);

    std::vector<geometry_msgs::PointStamped> arm_pts =
              arm_model_->project(data_, *offsets_);

    cv::Mat points;
//    std::cout << camera_pts.size() <<std::endl;
//    std::cout << arm_pts.size() << std::endl;     
   //cv::Mat some_1 = cv::Mat::zeros( arm_pts.size(), CV_32FC1); 
    //cv::Mat some_2 = cv::Mat::zeros( arm_pts.size(), CV_32FC1);
    //cv::Mat some_3 = cv::Mat::zeros( arm_pts.size(), CV_32FC1);
    for(size_t i =0; i<arm_pts.size(); i++)
    {
      //points.at<cv::Vec3f>(i,0)[0]= arm_pts[i].point.x;
      //points.at<cv::Vec3f>(i,0)[1]= arm_pts[i].point.y;
      //points.at<cv::Vec3f>(i,0)[2]= arm_pts[i].point.z;
      cv::Vec3f V(arm_pts[i].point.x,arm_pts[i].point.y, arm_pts[i].point.z);
      points.push_back(V);

    }

    cv::Vec3f normal = ( points.at<cv::Vec3f>(2,0) -  points.at<cv::Vec3f>(0,0)).cross( points.at<cv::Vec3f>(1,0) -  points.at<cv::Vec3f>(0,0));
//    cv::Vec3f normal = (rgb_points.at<cv::Vec3f>(2,0) - rgb_points.at<cv::Vec3f>(0,0)).cross(rgb_points.at<cv::Vec3f>(1,0)- rgb_points.at<cv::Vec3f>(0,0));
    cv::Vec4f plane;
    float distance = sqrt (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    plane[0] = normal[0] / distance;
    plane[1] = normal[1] / distance;
    plane[2] = normal[2] /distance;
    plane[3] = - points.at<cv::Vec3f>(1,0).dot(normal/distance);

    cv::Mat points_planar;
    for(size_t i =0; i<arm_pts.size(); i++)
    {
      cv::Point V(arm_pts[i].point.x,arm_pts[i].point.y);
      points_planar.push_back(V);
    }

    std::vector<cv::Point> hull;
    cv::Point closest_point;
    cv::convexHull(points_planar, hull, false);
    // Compute residuals
//    std::cout << "****"<<std::endl;
    for (size_t i = 0; i < camera_pts.size() ; ++i)
    {
      double dist = cv::pointPolygonTest(hull, cv::Point(camera_pts[i].point.x, camera_pts[i].point.y), false);
      float min_dist=10000;
      if (dist<0)    
      {
        for(size_t j =0; j < hull.size() ; j++)
        {
          float distance = pow((hull[j].x - camera_pts[i].point.x),2) + pow((hull[j].y - camera_pts[i].point.y),2);
          if( distance< min_dist)
          {
            min_dist = distance;
            closest_point = hull[j];
          }
          else
          {
            closest_point.x = 0;
            closest_point.y = 0;
            camera_pts[i].point.x = 0;
            camera_pts[i].point.y = 0;
            camera_pts[i].point.z = 0;

          }
        }
      } 
     /* else
      {
        closest_point.x = 0;
        closest_point.y = 0;
        camera_pts[i].point.x = 0;
        camera_pts[i].point.y = 0;
        camera_pts[i].point.z = 0;

      } */
      
      if(isnan(camera_pts[i].point.x) || isnan(camera_pts[i].point.y) || isnan(camera_pts[i].point.z))
      {
        camera_pts[i].point.x = 0;
        camera_pts[i].point.y = 0;
        camera_pts[i].point.z = 0;
      }
      //std::cout << camera_pts[i].point.x << "\t" << closest_point.x << std::endl;
      //std::cout << camera_pts[i].point.y << "\t" << closest_point.y << std::endl;

      residuals[(3*i)+0] = camera_pts[i].point.x - closest_point.x;
      residuals[(3*i)+1] = camera_pts[i].point.y - closest_point.y;
      residuals[(3*i)+2] = (camera_pts[i].point.x*plane[0] + camera_pts[i].point.y*plane[1] + camera_pts[i].point.z*plane[2] + plane[3]);// / sqrt(pow(plane[0],2) + pow(plane[1],2) + pow(plane[2],2));
  //    std::cout <<dist<<std::endl;
        // if camera_pts is in base frame
//        residuals[i] = (camera_pts[i].point.x*plane[0] + camera_pts[i].point.y*plane[1] + camera_pts[i].point.z*plane[2] + plane[3]);
    }
    return true;
  }

  static ceres::CostFunction* Create(Camera3dModel* camera_model,
                                     ChainModel* arm_model,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::CalibrationData& data)
  {
    ceres::DynamicNumericDiffCostFunction<GripperDepthError> * func;
    func = new ceres::DynamicNumericDiffCostFunction<GripperDepthError>(
                    new GripperDepthError(camera_model, arm_model,offsets, data ));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(data.observations[0].features.size()*3);

    return static_cast<ceres::CostFunction*>(func);
  }

  Camera3dModel * camera_model_;
  ChainModel * arm_model_;
  CalibrationOffsetParser * offsets_;
  robot_calibration_msgs::CalibrationData data_;
};
}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_GRIPPER_DEPTH_FINDER_H
