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

#include "mapvista_map_loader/map_loader_component.hpp"

namespace mapvista_map_loader
{

MapLoader::MapLoader(const rclcpp::NodeOptions & options)
: Node("map_loader_node", options)
{
  // Set parameters
  global_frame_id_ = this->declare_parameter<std::string>("global_frame_id", "map");

  // Create publisher and subscriber
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  pub_pcd_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pcd_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  srv_load_pcd_map_ = this->create_service<mapvista_map_msgs::srv::LoadPcdMap>(
    "/load_pcd_map", std::bind(&MapLoader::loadPcdMapCallback, this, _1, _2, _3),
    rclcpp::QoS(rclcpp::ServicesQoS()));

  // Load pcd_map
  const auto pcd_map_file = this->declare_parameter<std::string>("pcd_map_file", "");
  const auto pcd_map_type = this->declare_parameter<std::string>("pcd_map_type", "XYZI");
  if (pcd_map_file != "") {
    try {
      auto pcd_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
      loadPcdMap(pcd_map_file, pcd_map_type, pcd_msg_ptr);
      int pcd_subscription_count = pub_pcd_map_->get_subscription_count() +
        pub_pcd_map_->get_intra_process_subscription_count();
      if (pcd_subscription_count > 0) {
        pub_pcd_map_->publish(std::move(pcd_msg_ptr));
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
  }
}

MapLoader::~MapLoader()
{
}

void MapLoader::loadPcdMapCallback(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<mapvista_map_msgs::srv::LoadPcdMap::Request> req,
  std::shared_ptr<mapvista_map_msgs::srv::LoadPcdMap::Response> res)
{
  try {
    auto pcd_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    loadPcdMap(req->map_file, req->map_type, pcd_msg_ptr);
    int pcd_subscription_count =
      pub_pcd_map_->get_subscription_count() + pub_pcd_map_->get_intra_process_subscription_count();
    if (pcd_subscription_count > 0) {
      pub_pcd_map_->publish(std::move(pcd_msg_ptr));
    }
    res->result = true;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    res->result = false;
  }
}

void MapLoader::loadPcdMap(
  const std::string & map_file, const std::string & map_type,
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr)
{
  if (map_type == "XYZ") {
    if (this->loadPcdFile<pcl::PointXYZ>(map_file, pcd_msg_ptr)) {
      RCLCPP_INFO(this->get_logger(), "[PCDMap] Loaded pcd_map with PointXYZ type.");
    } else {
      throw std::runtime_error("[PCDMap] Failed to load pcd file.");
    }
  } else if (map_type == "XYZRGB") {
    if (this->loadPcdFile<pcl::PointXYZRGB>(map_file, pcd_msg_ptr)) {
      RCLCPP_INFO(this->get_logger(), "[PCDMap] Loaded pcd_map with PointXYZRGB type.");
    } else {
      throw std::runtime_error("[PCDMap] Failed to load pcd file.");
    }
  } else if (map_type == "XYZI") {
    if (this->loadPcdFile<pcl::PointXYZI>(map_file, pcd_msg_ptr)) {
      RCLCPP_INFO(this->get_logger(), "[PCDMap] Loaded pcd_map with PointXYZI type.");
    } else {
      throw std::runtime_error("[PCDMap] Failed to load pcd file.");
    }
  } else {
    throw std::runtime_error("[PCDMap] Invalid map type for pcd map.");
  }
}

template <typename T>
bool MapLoader::loadPcdFile(
  const std::string & map_file, std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr)
{
  auto map_cloud_ptr = std::make_shared<pcl::PointCloud<T>>();

  if (pcl::io::loadPCDFile<T>(map_file, *map_cloud_ptr) == -1) {
    return false;
  }

  pcl::toROSMsg(*map_cloud_ptr, *pcd_msg_ptr);
  pcd_msg_ptr->header.stamp = this->now();
  pcd_msg_ptr->header.frame_id = global_frame_id_;
  return true;
}

}  // namespace mapvista_map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mapvista_map_loader::MapLoader)
