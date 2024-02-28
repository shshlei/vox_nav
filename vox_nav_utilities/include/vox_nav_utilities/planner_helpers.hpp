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

#ifndef VOX_NAV_UTILITIES__PLANNER_HELPERS_HPP_
#define VOX_NAV_UTILITIES__PLANNER_HELPERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

// OMPL BASE
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>

// OCTOMAP
#include <octomap/octomap.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <map>
#include <vector>
#include <memory>
#include <string>

namespace vox_nav_utilities
{
void initializeSelectedPlanner(ompl::base::PlannerPtr & planner,
  const std::string & selected_planner_name, const ompl::base::SpaceInformationPtr & si, const rclcpp::Logger & logger);

// Get the Nearst Node to given state object
geometry_msgs::msg::PoseStamped getNearstNode(const geometry_msgs::msg::PoseStamped & state,
  const std::shared_ptr<octomap::OcTree> & nodes_octree);

float distance(float x, float y, float z, const pcl::PointCloud<pcl::PointXYZ>::Ptr & elevated_cloud);

float distance(float x, float y, float z, const pcl::PointCloud<pcl::PointSurfel>::Ptr & elevated_surfel_cloud);

void determineValidNearestStartGoal(geometry_msgs::msg::PoseStamped & nearest_valid_start, geometry_msgs::msg::PoseStamped & nearest_valid_goal,
  const geometry_msgs::msg::PoseStamped & actual_start, const geometry_msgs::msg::PoseStamped & actual_goal, const pcl::PointCloud<pcl::PointXYZ>::Ptr & elevated_cloud);

void determineValidNearestStartGoal(geometry_msgs::msg::PoseStamped & nearest_valid_start, geometry_msgs::msg::PoseStamped & nearest_valid_goal,
  const geometry_msgs::msg::PoseStamped & actual_start, const geometry_msgs::msg::PoseStamped & actual_goal, const pcl::PointCloud<pcl::PointSurfel>::Ptr & elevated_surfel_cloud);

void fillSuperVoxelMarkersfromAdjacency(const std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> & supervoxel_clusters,
  const std::multimap<std::uint32_t, std::uint32_t> & supervoxel_adjacency, const std_msgs::msg::Header & header,
  visualization_msgs::msg::MarkerArray & marker_array);

geometry_msgs::msg::PoseStamped getLinearInterpolatedPose(const geometry_msgs::msg::PoseStamped a, const geometry_msgs::msg::PoseStamped b);

void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> & path,
  const geometry_msgs::msg::Vector3 & marker_scale,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & plan_publisher,
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & nav_path_publisher,
  const bool & publish_segment_ids = true,
  const std::string & robot_mesh_path = "");

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__PLANNER_HELPERS_HPP_
