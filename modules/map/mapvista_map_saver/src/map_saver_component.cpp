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

#include "mapvista_map_saver/map_saver_component.hpp"

namespace mapvista_map_saver
{

MapSaver::MapSaver(const rclcpp::NodeOptions & options)
: Node("map_saver_node", options),
  subscribed_octomap_(false),
  subscribed_pcd_(false)
{
  // Create subscriber and service server
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MapSaver::octomapCallback, this, _1));
  sub_pcd_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pcd_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MapSaver::pcdMapCallback, this, _1));
  srv_save_map_ = this->create_service<mapvista_map_msgs::srv::SaveMap>(
    "/save_map", std::bind(&MapSaver::saveMapCallback, this, _1, _2, _3));
}

MapSaver::~MapSaver()
{
}

void MapSaver::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  octomap_msg_ptr_ = msg;
  subscribed_octomap_ = true;
}

void MapSaver::pcdMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcd_msg_ptr_ = msg;
  subscribed_pcd_ = true;
}

void MapSaver::saveMapCallback(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<mapvista_map_msgs::srv::SaveMap::Request> req,
  std::shared_ptr<mapvista_map_msgs::srv::SaveMap::Response> res)
{
  if (!subscribed_octomap_) {
    RCLCPP_ERROR(this->get_logger(), "[OCTOMAP_DOES_NOT_EXIST] Octomap is not subscribed.");
    res->result = false;
    return;
  }
  if (!subscribed_pcd_) {
    RCLCPP_ERROR(this->get_logger(), "[PCD_MAP_DOES_NOT_EXIST] PCD map is not subscribed.");
    res->result = false;
    return;
  }

  if (req->map_yaml_file == "") {
    RCLCPP_ERROR(this->get_logger(), "[INVALID_REQUEST] map_yaml_file is empty.");
    res->result = false;
    return;
  }
  if (req->octomap_suffix != "bt" && req->octomap_suffix != "ot") {
    RCLCPP_ERROR(this->get_logger(), "[INVALID_REQUEST] octomap_type is not bt or ot.");
    res->result = false;
    return;
  }

  try {
    saveMap(req->map_yaml_file, req->pcd_map_type, req->octomap_suffix, req->pcd_binary_mode);
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failure to save map: %s", e.what());
    res->result = false;
    return;
  }

  res->result = true;
}

void MapSaver::saveMap(
  const std::string & map_yaml_file, const std::string & pcd_map_type,
  const std::string & octomap_suffix, bool pcd_binary_mode)
{
  const auto suffix = map_yaml_file.substr(map_yaml_file.length() - 5, 5);
  auto map_file_name = map_yaml_file.substr(0, map_yaml_file.length() - 5);
  if (suffix != ".yaml") {
    throw std::runtime_error("[INVALID_REQUEST] map_yaml_file is not yaml.");
  }
  const auto pcd_map_file = map_file_name + ".pcd";
  const auto octomap_file = map_file_name + "." + octomap_suffix;

  try {
    // Save yaml file
    saveMapYaml(map_yaml_file, pcd_map_file, pcd_map_type, octomap_file);

    // Save pcd map
    savePcdMap(pcd_map_file, pcd_map_type, pcd_binary_mode);

    // Save octomap
    saveOctomap(octomap_file, octomap_suffix);
  } catch (const std::runtime_error & e) {
    throw e;
  }
}

void MapSaver::saveMapYaml(
  const std::string & map_yaml_file, const std::string & pcd_map_file,
  const std::string & pcd_map_type, const std::string & octomap_file)
{
  YAML::Node map_yaml_node;
  map_yaml_node["pcd"]["map_file"] = pcd_map_file;
  map_yaml_node["pcd"]["map_type"] = pcd_map_type;
  map_yaml_node["octomap"]["map_file"] = octomap_file;
  map_yaml_node["octomap"]["resolution"] = octomap_msg_ptr_->resolution;

  YAML::Emitter map_yaml_out;
  map_yaml_out << map_yaml_node;

  std::ofstream map_yaml(map_yaml_file);
  map_yaml << map_yaml_out.c_str();
  map_yaml.close();
}

void MapSaver::savePcdMap(
  const std::string & pcd_map_file, const std::string & pcd_map_type, bool binary_mode)
{
  if (pcd_map_type == "XYZ") {
    if (this->savePcdFile<pcl::PointXYZ>(pcd_map_file, binary_mode)) {
      RCLCPP_INFO(this->get_logger(), "Saved pcd_map with PointXYZ type.");
    } else {
      throw std::runtime_error("[INVALID_PCD_MAP] Failed to save pcd file.");
    }
  } else if (pcd_map_type == "XYZRGB") {
    if (this->savePcdFile<pcl::PointXYZRGB>(pcd_map_file, binary_mode)) {
      RCLCPP_INFO(this->get_logger(), "Saved pcd_map with PointXYZ type.");
    } else {
      throw std::runtime_error("[INVALID_PCD_MAP] Failed to save pcd file.");
    }
  } else if (pcd_map_type == "XYZI") {
    if (this->savePcdFile<pcl::PointXYZI>(pcd_map_file, binary_mode)) {
      RCLCPP_INFO(this->get_logger(), "Saved pcd_map with PointXYZ type.");
    } else {
      throw std::runtime_error("[INVALID_PCD_MAP] Failed to save pcd file.");
    }
  } else {
    throw std::runtime_error("[INVAID_PCD_MAP] Invalid map type for pcd map.");
  }
}

void MapSaver::saveOctomap(const std::string & octomap_file, const std::string & octomap_suffix)
{
  std::unique_ptr<octomap::AbstractOcTree> tree{octomap_msgs::msgToMap(*octomap_msg_ptr_)};
  std::unique_ptr<octomap::AbstractOccupancyOcTree> octree;
  if (tree) {
    octree = std::unique_ptr<octomap::AbstractOccupancyOcTree>(
      dynamic_cast<octomap::AbstractOccupancyOcTree *>(tree.release()));
  } else {
    throw std::runtime_error("Error creating octree from received message.");
  }

  if (octree) {
    if (octomap_suffix == "bt") {
      if (!octree->writeBinary(octomap_file)) {
        std::string error_message = "Error writing to file " + octomap_file;
        throw std::runtime_error(error_message);
      }
    } else if (octomap_suffix == "ot") {
      if (!octree->write(octomap_file)) {
        std::string error_message = "Error writing to file " + octomap_file;
        throw std::runtime_error(error_message);
      }
    } else {
      throw std::runtime_error("Unknown file extension, must be either .bt or .ot");
    }
  } else {
    throw std::runtime_error("Error reading OcTree from stream.");
  }
}

template <typename T>
bool MapSaver::savePcdFile(const std::string & pcd_map_file, bool binary_mode)
{
  auto map_cloud_ptr = std::make_shared<pcl::PointCloud<T>>();
  pcl::fromROSMsg(*pcd_msg_ptr_, *map_cloud_ptr);

  if (binary_mode) {
    if (pcl::io::savePCDFileBinary<T>(pcd_map_file, *map_cloud_ptr) == -1) {
      return false;
    }
  } else {
    if (pcl::io::savePCDFileASCII<T>(pcd_map_file, *map_cloud_ptr) == -1) {
      return false;
    }
  }
  return true;
}

}  // namespace mapvista_map_saver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mapvista_map_saver::MapSaver)
