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

#include "mapvista_map_converter/map_converter_component.hpp"

namespace mapvista_map_converter
{

MapConverter::MapConverter(const rclcpp::NodeOptions & options)
: Node("map_converter", options)
{
  // Declare ROS 2 parameters
  resolution_ = this->declare_parameter<double>("octomap_resolution", 0.1);
  map_type_ = this->declare_parameter<std::string>("map_type", "XYZI");

  // Create publisher and subscription
  using std::placeholders::_1;
  pub_octomap_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    "/octomap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sub_pcd_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pcd_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MapConverter::pcdMapCallback, this, _1));
}

MapConverter::~MapConverter()
{
}

void MapConverter::pcdMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr)
{
  auto octomap_msg_ptr = std::make_unique<octomap_msgs::msg::Octomap>();

  if (map_type_ == "XYZ") {
    mapConversion<pcl::PointXYZ>(pcd_msg_ptr, octomap_msg_ptr);
  } else if (map_type_ == "XYZRGB") {
    mapConversion<pcl::PointXYZRGB>(pcd_msg_ptr, octomap_msg_ptr);
  } else if (map_type_ == "XYZI") {
    mapConversion<pcl::PointXYZI>(pcd_msg_ptr, octomap_msg_ptr);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid map type for pcd map.");
    return;
  }

  octomap_msg_ptr->header.stamp = pcd_msg_ptr->header.stamp;
  octomap_msg_ptr->header.frame_id = pcd_msg_ptr->header.frame_id;
  pub_octomap_->publish(std::move(octomap_msg_ptr));
}

template <typename T>
void MapConverter::mapConversion(
  const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr,
  std::unique_ptr<octomap_msgs::msg::Octomap> & octomap_msg_ptr)
{
  pcl::PCLPointCloud2 pcl_pcd;
  pcl_conversions::toPCL(*pcd_msg_ptr, pcl_pcd);
  typename pcl::PointCloud<T>::Ptr pcl_cloud_ptr(new pcl::PointCloud<T>);
  pcl::fromPCLPointCloud2(pcl_pcd, *pcl_cloud_ptr);

  octomap::OcTree tree(resolution_);
  for (const auto & point : pcl_cloud_ptr->points) {
    tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
  }
  tree.updateInnerOccupancy();

  octomap_msgs::fullMapToMsg(tree, *octomap_msg_ptr);
}

}  // namespace mapvista_map_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mapvista_map_converter::MapConverter)
