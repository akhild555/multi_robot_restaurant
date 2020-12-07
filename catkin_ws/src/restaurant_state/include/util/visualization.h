#pragma once
// Copyright 2019 - 2020 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ========================================================================
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <tuple>
#include <vector>


namespace visualization {

inline visualization_msgs::Marker MakeCylinder(const std::vector<float> position,
                                               const float radius,
                                               const float height,
                                               const std::string& frame_id,
                                               const std::string& ns,
                                               const float r,
                                               const float g,
                                               const float b,
                                               const float alpha,
                                               const float z = 0,
                                               const bool task_type = true) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  if(task_type==true){
    marker.type = visualization_msgs::Marker::CYLINDER;
  }
  else{
    marker.type = visualization_msgs::Marker::CUBE;
  }
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 2 * radius;
  marker.scale.y = 2 * radius;
  marker.scale.z = height;
  marker.pose.position.x = position[0];
  marker.pose.position.y = position[1];
  marker.pose.position.z = z;
  marker.pose.orientation.w = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.color.a = alpha;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  return marker;
}

}  // namespace visualization
