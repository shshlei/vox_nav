// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_PLANNING__PLUGINS__SE2_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__SE2_PLANNER_HPP_

#include "vox_nav_planning/planner_core.hpp"

#include <vox_nav_msgs/srv/get_traversability_map.hpp>

#include <fcl/narrowphase/collision_object.h>
#include <octomap/octomap.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace vox_nav_planning
{
class SE2Planner : public vox_nav_planning::PlannerCore
{
public:
  SE2Planner() = default;

  virtual ~SE2Planner() = default;

  void initialize(rclcpp::Node * parent) override;

  std::vector<geometry_msgs::msg::PoseStamped> createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override;

  bool isStateValid(const ompl::base::State * state) override;

  void setupMap() override;

protected:
  rclcpp::Client<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr get_traversability_map_client_;

  rclcpp::Logger logger_{rclcpp::get_logger("se2_planner")};

  // Distance of robot above the flat ground plane, NOte
  // this planner is supposed to operate on even flat ground
  // z_elevation is constant everywhere
  double z_elevation_;

  // curve radius for reeds and dubins only
  double rho_;

  ompl::base::StateSpacePtr state_space_;

  std::shared_ptr<ompl::base::RealVectorBounds> se2_bounds_;

  std::shared_ptr<fcl::CollisionObjectf> original_octomap_collision_object_;

  std::shared_ptr<fcl::CollisionObjectf> robot_collision_object_;

  // global mutex to guard octomap
  std::mutex octomap_mutex_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__SE2_PLANNER_HPP_
