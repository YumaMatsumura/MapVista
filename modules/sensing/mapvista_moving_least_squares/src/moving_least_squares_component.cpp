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

#include "mapvista_moving_least_squares/moving_least_squares_component.hpp"

namespace mapvista_moving_least_squares
{

MovingLeastSquares::MovingLeastSquares(const rclcpp::NodeOptions & options)
: Node("moving_least_squares", options)
{
  polynomial_order_ = this->declare_parameter<int>("polynomial_order", 2);
  search_radius_ = this->declare_parameter<double>("search_radius", 0.03);

  // Create publisher and subscription
  using std::placeholders::_1;
  pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/output_pcd", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sub_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/input_pcd", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MovingLeastSquares::pcdCallback, this, _1));
}

MovingLeastSquares::~MovingLeastSquares()
{
}

void MovingLeastSquares::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr)
{
  auto pcd_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto filtered_pcd_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  pcl::fromROSMsg(*pcd_msg_ptr, *pcd_cloud_ptr);
  mls.setComputeNormals(true);
  mls.setInputCloud(pcd_cloud_ptr);
  mls.setPolynomialOrder(polynomial_order_);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(search_radius_);

  mls.process(mls_points);

  pcl::toROSMsg(mls_points, *filtered_pcd_msg_ptr);
  filtered_pcd_msg_ptr->header = pcd_msg_ptr->header;
  pub_pcd_->publish(std::move(filtered_pcd_msg_ptr));
}

}  // namespace mapvista_moving_least_squares

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mapvista_moving_least_squares::MovingLeastSquares)
