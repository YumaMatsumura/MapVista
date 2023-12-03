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

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "mapvista_map_msgs/srv/save_map.hpp"
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

#include "yaml-cpp/yaml.h"

namespace mapvista_map_saver
{

class MapSaver : public rclcpp::Node
{
public:
  explicit MapSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapSaver();

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void pcdMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void saveMapCallback(
    [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<mapvista_map_msgs::srv::SaveMap::Request> req,
    std::shared_ptr<mapvista_map_msgs::srv::SaveMap::Response> res);
  void saveMap(
    const std::string & map_yaml_file, const std::string & pcd_map_type,
    const std::string & octomap_suffix, bool pcd_binary_mode);
  void saveMapYaml(
    const std::string & map_yaml_file, const std::string & pcd_map_file,
    const std::string & pcd_map_type, const std::string & octomap_file);
  void savePcdMap(
    const std::string & pcd_map_file, const std::string & pcd_map_type, bool binary_mode);
  void saveOctomap(const std::string & octomap_file, const std::string & octomap_suffix);
  template <typename T>
  bool savePcdFile(const std::string & pcd_map_file, bool binary_mode);

  // ROS 2 subscriber and service server
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_map_;
  rclcpp::Service<mapvista_map_msgs::srv::SaveMap>::SharedPtr srv_save_map_;
  // ROS 2 messages
  octomap_msgs::msg::Octomap::SharedPtr octomap_msg_ptr_;
  sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr_;
  // Flags
  bool subscribed_octomap_;
  bool subscribed_pcd_;
};

}  // namespace mapvista_map_saver
