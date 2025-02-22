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

#ifndef VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include <vox_nav_utilities/elevation_state_space.hpp>
#include <vox_nav_msgs/srv/get_traversability_map.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fcl/narrowphase/collision_object.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <memory>
#include <string>
#include <vector>

namespace vox_nav_planning
{
// The method creates supervoxel adjacency graph from the point cloud
// The A* search is performed on this graph to find the optimal elevation path from start to goal.
class OptimalElevationPlanner : public vox_nav_planning::PlannerCore
{
public:
  OptimalElevationPlanner();

  ~OptimalElevationPlanner();

  void initialize(rclcpp::Node * parent) override;

  std::vector<geometry_msgs::msg::PoseStamped> createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override;

  bool isStateValid(const ompl::base::State * state) override;

  void setupMap() override;

  void nodePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  ompl::base::ValidStateSamplerPtr allocValidStateSampler(const ompl::base::SpaceInformation * si);

  bool isEdgeinCollision(const pcl::PointXYZRGBA & a, const pcl::PointXYZRGBA & b);

protected:
  typedef std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> SuperVoxelClusters;

  rclcpp::Logger logger_{rclcpp::get_logger("optimal_elevation_planner")};

  // Get the traversability map from vox_nav_map_server
  rclcpp::Client<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr get_traversability_map_client_;

  pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_cloud_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr elevated_traversable_cloud_;

  ompl::base::StateSpacePtr state_space_;

  std::shared_ptr<ompl::base::RealVectorBounds> z_bounds_;

  std::shared_ptr<ompl::base::RealVectorBounds> se2_bounds_;

  // SuperVoxel Clustering variables
  // https://pcl.readthedocs.io/en/latest/supervoxel_clustering.html#supervoxel-clustering
  // boost graph is constructed through supervoxels of elevated surfels
  // Optimal planning basing in Astar is perfromed on top of this graph
  // refer to PCL supervoxel_clustering for more details on algorithm
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr super_voxel_adjacency_marker_pub_;

  SuperVoxelClusters supervoxel_clusters_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_clusters_pub_;

  bool supervoxel_disable_transform_;

  float supervoxel_resolution_;

  float supervoxel_seed_resolution_;

  float supervoxel_color_importance_;

  float supervoxel_spatial_importance_;

  float supervoxel_normal_importance_;

  float distance_penalty_weight_;

  float elevation_penalty_weight_;

  std::string graph_search_method_;  // astar ? , diskstra ?

  ompl::base::ElevationStateSpace::SE2StateType se2_space_type_;

  // curve radius for reeds and dubins only
  double rho_;

  // octomap acquired from original PCD map
  std::shared_ptr<fcl::CollisionObjectf> original_octomap_collision_object_;

  std::shared_ptr<fcl::CollisionObjectf> robot_collision_object_;

  // Better t keep this parameter consistent with map_server, 0.2 is a OK default fo this
  double octomap_voxel_size_;

  // global mutex to guard octomap
  std::mutex octomap_mutex_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_
