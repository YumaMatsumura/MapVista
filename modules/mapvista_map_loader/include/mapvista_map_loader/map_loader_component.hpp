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

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

#include "mapvista_map_msgs/srv/load_pcd_map.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace mapvista_map_loader
{

class MapLoader : public rclcpp::Node
{
public:
  explicit MapLoader(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapLoader();

private:
  void loadPcdMapCallback(
    [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<mapvista_map_msgs::srv::LoadPcdMap::Request> req,
    std::shared_ptr<mapvista_map_msgs::srv::LoadPcdMap::Response> res);
  void loadPcdMap(
    const std::string & map_file, const std::string & map_type,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr);
  template <typename T>
  bool loadPcdFile(
    const std::string & map_file, std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr);

  // ROS 2 publisher and service server
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_map_;
  rclcpp::Service<mapvista_map_msgs::srv::LoadPcdMap>::SharedPtr srv_load_pcd_map_;
  // Parameters
  std::string global_frame_id_;
};

}  // namespace mapvista_map_loader
