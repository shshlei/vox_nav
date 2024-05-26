// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_map_server/map_manager.hpp"
#include "vox_nav_map_server/map_manager_helpers.hpp"
#include <vox_nav_utilities/pcl_helpers.hpp>

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.h>
#include <octomap_msgs/conversions.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace vox_nav_map_server
{
MapManager::MapManager()
: MapManagerBase("vox_nav_map_manager_rclcpp_node")
{
  // initialize shared pointers asap
  pcd_map_gps_pose_ = std::make_shared<vox_nav_msgs::msg::OrientedNavSatFix>();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  utm_frame_id_ = declare_parameter("utm_frame_id", "utm");
  pcd_map_gps_pose_->position.latitude = declare_parameter("map_datum.latitude", 49.0);
  pcd_map_gps_pose_->position.longitude = declare_parameter("map_datum.longitude", 3.0);
  pcd_map_gps_pose_->position.altitude = declare_parameter("map_datum.altitude", 0.5);
  pcd_map_gps_pose_->orientation.x = declare_parameter("map_datum.quaternion.x", 0.0);
  pcd_map_gps_pose_->orientation.y = declare_parameter("map_datum.quaternion.y", 0.0);
  pcd_map_gps_pose_->orientation.z = declare_parameter("map_datum.quaternion.z", 0.0);
  pcd_map_gps_pose_->orientation.w = declare_parameter("map_datum.quaternion.w", 1.0);

  // service hooks for robot localization fromll service
  robot_localization_fromLL_client_node_ = std::make_shared<rclcpp::Node>("map_manager_fromll_client_node");
  robot_localization_fromLL_client_ = robot_localization_fromLL_client_node_->create_client<robot_localization::srv::FromLL>("/fromLL");

  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&MapManager::timerCallback, this));
}

void MapManager::timerCallback()
{
  // Since this is static map we need to georefence this only once not each time
  std::call_once(
    configure_map_once_, [this]() {
      while (!tf_buffer_->canTransform(utm_frame_id_, map_frame_id_, rclcpp::Time(0))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(this->get_logger(), "Waiting for %s to %s Transform to be available.",
          utm_frame_id_.c_str(), map_frame_id_.c_str());
      }
      RCLCPP_INFO(get_logger(),
        "Configuring pcd map with given parameters,"
        " But the map and octomap will be published at %i frequency rate",
        octomap_publish_frequency_);

      transfromPCDfromGPS2Map();
      preProcessPCDMap();
      regressCosts();
      handleOriginalOctomap();
      RCLCPP_INFO(get_logger(), "Georeferenced given map, ready to publish");

      map_configured_ = true;
    });
  publishMapVisuals();
}

void MapManager::transfromPCDfromGPS2Map()
{
  auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
  auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
  request->ll_point.latitude = pcd_map_gps_pose_->position.latitude;
  request->ll_point.longitude = pcd_map_gps_pose_->position.longitude;
  request->ll_point.altitude = pcd_map_gps_pose_->position.altitude;

  while (!robot_localization_fromLL_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the /fromLL service.Exiting");
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "/fromLL service not available, waiting and trying again");
  }

  auto result_future = robot_localization_fromLL_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
        robot_localization_fromLL_client_node_,
        result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "/fromLL service call failed");
  }
  auto result = result_future.get();
  response->map_point = result->map_point;

  // The translation from static_map origin to map is basically inverse of this transform
  tf2::Transform static_map_to_map_transfrom;
  static_map_to_map_transfrom.setOrigin(
    tf2::Vector3(
      response->map_point.x,
      response->map_point.y,
      response->map_point.z));

  tf2::Quaternion static_map_quaternion;
  tf2::fromMsg(pcd_map_gps_pose_->orientation, static_map_quaternion);
  static_map_to_map_transfrom.setRotation(static_map_quaternion);

  geometry_msgs::msg::TransformStamped stamped;
  stamped.child_frame_id = "static_map";
  stamped.header.frame_id = "map";
  stamped.header.stamp = this->get_clock()->now();
  stamped.transform.rotation = pcd_map_gps_pose_->orientation;
  geometry_msgs::msg::Vector3 translation;
  translation.x = response->map_point.x;
  translation.y = response->map_point.y;
  translation.z = response->map_point.z;
  stamped.transform.translation = translation;
  static_transform_broadcaster_->sendTransform(stamped);

  pcl_ros::transformPointCloud(*pcd_map_pointcloud_, *pcd_map_pointcloud_, static_map_to_map_transfrom);
}
}  // namespace vox_nav_map_server

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto map_manager_node = std::make_shared<vox_nav_map_server::MapManager>();
  rclcpp::spin(map_manager_node);
  rclcpp::shutdown();
  return 0;
}
