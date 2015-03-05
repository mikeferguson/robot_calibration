/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOT_CALIBRATION_PCL_CONVERSIONS_H
#define ROBOT_CALIBRATION_PCL_CONVERSIONS_H

/** @brief PCL never works. Try to fix a bit of that. */
namespace pcl_broke_again
{

/*
 * This has already been submitted to pcl_conversions, but not released.
 */

template<typename CloudT> void
toROSMsg (const CloudT& cloud, sensor_msgs::Image& msg)
{
  // Ease the user's burden on specifying width/height for unorganized datasets
  if (cloud.width == 0 && cloud.height == 0)
    throw std::runtime_error("Needs to be a dense like cloud!!");
  else
  {
    if (cloud.points.size () != cloud.width * cloud.height)
      throw std::runtime_error("The width and height do not match the cloud size!");
    msg.height = cloud.height;
    msg.width = cloud.width;
  }

  // sensor_msgs::image_encodings::BGR8;
  msg.encoding = "bgr8";
  msg.step = msg.width * sizeof (uint8_t) * 3;
  msg.data.resize (msg.step * msg.height);
  for (size_t y = 0; y < cloud.height; y++)
  {
    for (size_t x = 0; x < cloud.width; x++)
    {
      uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
      memcpy (pixel, &cloud (x, y).rgb, 3 * sizeof(uint8_t));
    }
  }
}

}

#endif  // ROBOT_CALIBRATION_PCL_CONVERSIONS_H