// Copyright (c) 2021 Norwegian University of Life Sciences Fetullah Atas
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

#ifndef VOX_NAV_MAP_SERVER__MAP_MANAGER_HPP_
#define VOX_NAV_MAP_SERVER__MAP_MANAGER_HPP_

#include "vox_nav_map_server/map_manager_base.hpp"
#include <vox_nav_msgs/msg/oriented_nav_sat_fix.hpp>

#include <robot_localization/srv/from_ll.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace vox_nav_map_server
{
class MapManager : public MapManagerBase
{
public:

  MapManager();

  virtual ~MapManager() = default;

protected:

  /**
   * @brief Given PCD Map's GPS coordinate and heading,
   * this method aligns PCD Map to robots initial coordinates,
   * thats basically "map" frame published by robot_localization.
   * One must think PCD Map as a static map.
   *
   */
  void transfromPCDfromGPS2Map();

protected:

  void timerCallback();

  // Used to call a periodic callback function IOT publish octomap visuals
  rclcpp::TimerBase::SharedPtr timer_;

  // we need to align static map to map only once, since it is static !
  std::once_flag configure_map_once_;

protected:

  // clint node used for spinning the service callback of robot_localization_fromLL_client_
  rclcpp::Node::SharedPtr robot_localization_fromLL_client_node_;

  // robot_localization package provides a service to convert
  // lat,long,al GPS cooordinates to x,y,z map points
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr robot_localization_fromLL_client_;

  // we read gps coordinates of map from yaml
  vox_nav_msgs::msg::OrientedNavSatFix::SharedPtr pcd_map_gps_pose_;

  std::string utm_frame_id_;

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
};
}  // namespace vox_nav_map_server

#endif  // VOX_NAV_MAP_SERVER__MAP_MANAGER_HPP_
