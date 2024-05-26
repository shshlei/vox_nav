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

#ifndef VOX_NAV_MAP_SERVER__MAP_MANAGER_NO_GPS_HPP_
#define VOX_NAV_MAP_SERVER__MAP_MANAGER_NO_GPS_HPP_

#include "vox_nav_map_server/map_manager_base.hpp"

namespace vox_nav_map_server
{
class MapManagerNoGPS : public MapManagerBase
{
public:
  MapManagerNoGPS();

  virtual ~MapManagerNoGPS() = default;

protected:

  // Used to call a periodic callback function IOT publish octomap visuals
  rclcpp::TimerBase::SharedPtr timer_;

  void timerCallback();

  // we need to align static map to map only once, since it is static !
  std::once_flag configure_map_once_;
};
}  // namespace vox_nav_map_server

#endif  // VOX_NAV_MAP_SERVER__MAP_MANAGER_NO_GPS_HPP_
