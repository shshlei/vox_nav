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

#ifndef VOX_NAV_PLANNING__PLUGINS__ELEVATION_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__ELEVATION_PLANNER_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include <vox_nav_utilities/elevation_state_space.hpp>
#include <vox_nav_msgs/srv/get_traversability_map.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fcl/narrowphase/collision_object.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace vox_nav_planning
{

class ElevationPlanner : public vox_nav_planning::PlannerCore
{
public:
  ElevationPlanner() = default;

  virtual ~ElevationPlanner() = default;

  void initialize(rclcpp::Node * parent) override;

  bool isStateValid(const ompl::base::State * state) override;

  void setupMap() override;

  std::vector<geometry_msgs::msg::PoseStamped> createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("elevation_planner")};

  // global mutex to guard octomap
  std::mutex octomap_mutex_;

  rclcpp::Client<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr get_traversability_map_client_;

  pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_pointcloud_;

  std::shared_ptr<fcl::CollisionObjectf> original_octomap_collision_object_;

  std::shared_ptr<fcl::CollisionObjectf> robot_collision_object_;

  double min_surfel_distance_{0.0};

  double max_surfel_distance_{0.2};

  // OMPL
  ompl::base::StateSpacePtr state_space_;

  std::shared_ptr<ompl::base::RealVectorBounds> se2_bounds_;

  std::shared_ptr<ompl::base::RealVectorBounds> z_bounds_;

  ompl::base::ElevationStateSpace::SE2StateType se2_space_type_;

  // curve radius for reeds and dubins only
  double rho_;

  // Publisher Planner Nodes
  bool pub_nodes_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planner_nodes_pub_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__ELEVATION_PLANNER_HPP_
