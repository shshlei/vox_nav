// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_map_server/map_manager_no_gps.hpp"
#include "vox_nav_map_server/map_manager_helpers.hpp"
#include <vox_nav_utilities/pcl_helpers.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.h>
#include <octomap_msgs/conversions.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace vox_nav_map_server
{
MapManagerNoGPS::MapManagerNoGPS()
: MapManagerBase("vox_nav_map_manager_no_gps_rclcpp_node")
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&MapManagerNoGPS::timerCallback, this));
}

void MapManagerNoGPS::timerCallback()
{
  // Since this is static map we need to georefence this only once not each time
  std::call_once(configure_map_once_, [this]() {
    RCLCPP_INFO(get_logger(),
      "Configuring pcd map with given parameters,"
      " But the map and octomap will be published at %i frequency rate",
      octomap_publish_frequency_);

    preProcessPCDMap();
    regressCosts();
    handleOriginalOctomap();
    RCLCPP_INFO(get_logger(), "Georeferenced given map, ready to publish");

    map_configured_ = true;
  });
  publishMapVisuals();
}
}  // namespace vox_nav_map_server

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto map_manager_node = std::make_shared<vox_nav_map_server::MapManagerNoGPS>();
  rclcpp::spin(map_manager_node);
  rclcpp::shutdown();
  return 0;
}
