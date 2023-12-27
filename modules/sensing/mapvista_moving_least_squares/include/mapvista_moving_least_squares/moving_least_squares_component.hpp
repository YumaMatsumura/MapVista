// Copyright 2023 Yuma Matsumura All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace mapvista_moving_least_squares
{

class MovingLeastSquares : public rclcpp::Node
{
public:
  explicit MovingLeastSquares(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MovingLeastSquares();

private:
  void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr);

  // ROS 2 publisher and subscriber
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
  // Parameters
  int polynomial_order_;
  double search_radius_;
};

}  // namespace mapvista_moving_least_squares
