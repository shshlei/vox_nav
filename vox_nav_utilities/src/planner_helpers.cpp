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

#include "vox_nav_utilities/planner_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"

// OMPL GEOMETRIC
#include <ompl/geometric/planners/ase/BiASE.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>

#include <map>
#include <cmath>
#include <vector>
#include <memory>
#include <string>

namespace vox_nav_utilities
{
void initializeSelectedPlanner(ompl::base::PlannerPtr & planner, const std::string & selected_planner_name,
  const ompl::base::SpaceInformationPtr & si, const rclcpp::Logger & logger)
{
  if (selected_planner_name == std::string("BiASE")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::BiASE(si));
  }
  else if (selected_planner_name == std::string("BiASEstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::BiASEstar(si));
  }
  else if (selected_planner_name == std::string("RRT")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRT(si));
  }
  else if (selected_planner_name == std::string("RRTConnect")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si));
  }
  else if (selected_planner_name == std::string("PRMstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
  }
  else if (selected_planner_name == std::string("LazyPRMstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::LazyPRMstar(si));
  }
  else if (selected_planner_name == std::string("RRTstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
  }
  else if (selected_planner_name == std::string("RRTsharp")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(si));
  }
  else if (selected_planner_name == std::string("RRTXstatic")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTXstatic(si));
  }
  else if (selected_planner_name == std::string("InformedRRTstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::InformedRRTstar(si));
  }
  else if (selected_planner_name == std::string("BITstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::BITstar(si));
  }
  else if (selected_planner_name == std::string("ABITstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::ABITstar(si));
  }
  else if (selected_planner_name == std::string("AITstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::AITstar(si));
  }
  else if (selected_planner_name == std::string("CForest")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::CForest(si));
  }
  else if (selected_planner_name == std::string("LBTRRT")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::LBTRRT(si));
  }
  else if (selected_planner_name == std::string("SST")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::SST(si));
  }
  else if (selected_planner_name == std::string("TRRT")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::TRRT(si));
  }
  else if (selected_planner_name == std::string("SPARS")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::SPARS(si));
  }
  else if (selected_planner_name == std::string("SPARStwo")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::SPARStwo(si));
  }
  else if (selected_planner_name == std::string("FMT")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::FMT(si));
  }
  else if (selected_planner_name == std::string("BFMT")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::BFMT(si));
  }
  else if (selected_planner_name == std::string("AnytimePathShortening")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::AnytimePathShortening(si));
    auto aps_planner = std::make_shared<ompl::geometric::AnytimePathShortening>(si);
    for (std::size_t i = 0; i < 8; i++) {
      auto prm_planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
      aps_planner->addPlanner(prm_planner);
    }
    planner = ompl::base::PlannerPtr(aps_planner);
  }
  else {
    RCLCPP_WARN(logger, "Selected planner is not Found in available planners, using the default planner: RRTstar");
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
  }
}

geometry_msgs::msg::PoseStamped getNearstNode(const geometry_msgs::msg::PoseStamped & state, const std::shared_ptr<octomap::OcTree> & nodes_octree)
{
  auto nearest_node_pose = state;
  double dist = INFINITY;
  for (auto it = nodes_octree->begin(), end = nodes_octree->end(); it != end; ++it) {
    if (!nodes_octree->isNodeOccupied(*it)) continue;

    auto dist_to_crr_node = std::sqrt(std::pow(it.getCoordinate().x() - state.pose.position.x, 2) +
                                      std::pow(it.getCoordinate().y() - state.pose.position.y, 2) +
                                      std::pow(it.getCoordinate().z() - state.pose.position.z, 2));
    if (dist_to_crr_node >= dist) continue;

    dist = dist_to_crr_node;
    nearest_node_pose.pose.position.x = it.getCoordinate().x();
    nearest_node_pose.pose.position.y = it.getCoordinate().y();
    nearest_node_pose.pose.position.z = it.getCoordinate().z();
  }
  return nearest_node_pose;
}

float distance(float x, float y, float z, const pcl::PointCloud<pcl::PointXYZ>::Ptr & elevated_cloud)
{
  pcl::PointXYZ surfel;
  surfel.x = x;
  surfel.y = y;
  surfel.z = z;

  pcl::PointXYZ nearest_surfel = vox_nav_utilities::getNearstPoint<pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr>(surfel, elevated_cloud);
  double distance = std::sqrt(std::pow(surfel.x - nearest_surfel.x, 2) + std::pow(surfel.y - nearest_surfel.y, 2) + std::pow(surfel.z - nearest_surfel.z, 2));
  return distance;
}

float distance(float x, float y, float z, const pcl::PointCloud<pcl::PointSurfel>::Ptr & elevated_surfel_cloud)
{
  pcl::PointSurfel surfel;
  surfel.x = x;
  surfel.y = y;
  surfel.z = z;

  pcl::PointSurfel nearest_surfel = vox_nav_utilities::getNearstPoint<pcl::PointSurfel, pcl::PointCloud<pcl::PointSurfel>::Ptr>(surfel, elevated_surfel_cloud);
  float distance = (surfel.x - nearest_surfel.x) * nearest_surfel.normal_x + (surfel.y - nearest_surfel.y) * nearest_surfel.normal_y + (surfel.z - nearest_surfel.z) * nearest_surfel.normal_z;
  return distance;
}

void determineValidNearestStartGoal(geometry_msgs::msg::PoseStamped & nearest_valid_start, geometry_msgs::msg::PoseStamped & nearest_valid_goal,
  const geometry_msgs::msg::PoseStamped & actual_start, const geometry_msgs::msg::PoseStamped & actual_goal,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & elevated_cloud)
{
  pcl::PointXYZ start_nearest, goal_nearest;
  start_nearest.x = actual_start.pose.position.x;
  start_nearest.y = actual_start.pose.position.y;
  start_nearest.z = actual_start.pose.position.z;
  goal_nearest.x = actual_goal.pose.position.x;
  goal_nearest.y = actual_goal.pose.position.y;
  goal_nearest.z = actual_goal.pose.position.z;
  
  start_nearest = vox_nav_utilities::getNearstPoint<pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr>(start_nearest, elevated_cloud);
  goal_nearest = vox_nav_utilities::getNearstPoint<pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr>(goal_nearest, elevated_cloud);

  nearest_valid_start.pose.position.x = start_nearest.x;
  nearest_valid_start.pose.position.y = start_nearest.y;
  nearest_valid_start.pose.position.z = start_nearest.z;
  nearest_valid_goal.pose.position.x = goal_nearest.x;
  nearest_valid_goal.pose.position.y = goal_nearest.y;
  nearest_valid_goal.pose.position.z = goal_nearest.z;
}

void determineValidNearestStartGoal(geometry_msgs::msg::PoseStamped & nearest_valid_start, geometry_msgs::msg::PoseStamped & nearest_valid_goal,
  const geometry_msgs::msg::PoseStamped & actual_start, const geometry_msgs::msg::PoseStamped & actual_goal,
  const pcl::PointCloud<pcl::PointSurfel>::Ptr & elevated_surfel_cloud)
{
  pcl::PointSurfel start_nearest, goal_nearest;
  start_nearest.x = actual_start.pose.position.x;
  start_nearest.y = actual_start.pose.position.y;
  start_nearest.z = actual_start.pose.position.z;
  goal_nearest.x = actual_goal.pose.position.x;
  goal_nearest.y = actual_goal.pose.position.y;
  goal_nearest.z = actual_goal.pose.position.z;
  
  start_nearest = vox_nav_utilities::getNearstPoint<pcl::PointSurfel, pcl::PointCloud<pcl::PointSurfel>::Ptr>(start_nearest, elevated_surfel_cloud);
  goal_nearest = vox_nav_utilities::getNearstPoint<pcl::PointSurfel, pcl::PointCloud<pcl::PointSurfel>::Ptr>(goal_nearest, elevated_surfel_cloud);

  nearest_valid_start.pose.position.x = start_nearest.x;
  nearest_valid_start.pose.position.y = start_nearest.y;
  nearest_valid_start.pose.position.z = start_nearest.z;
  nearest_valid_goal.pose.position.x = goal_nearest.x;
  nearest_valid_goal.pose.position.y = goal_nearest.y;
  nearest_valid_goal.pose.position.z = goal_nearest.z;
}

void fillSuperVoxelMarkersfromAdjacency(const std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> & supervoxel_clusters,
  const std::multimap<std::uint32_t, std::uint32_t> & supervoxel_adjacency, const std_msgs::msg::Header & header,
  visualization_msgs::msg::MarkerArray & marker_array)
{
  int index = 0;
  // To make a graph of the supervoxel adjacency,
  // we need to iterate through the supervoxel adjacency multimap
  for (auto label_itr = supervoxel_adjacency.cbegin(); label_itr != supervoxel_adjacency.cend();) {
    // First get the label
    std::uint32_t supervoxel_label = label_itr->first;
    // Now get the supervoxel corresponding to the label
    auto supervoxel = supervoxel_clusters.at(supervoxel_label);

    visualization_msgs::msg::Marker line_strip;
    line_strip.header = header;
    line_strip.ns = "supervoxel_markers_ns";
    line_strip.id = index;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.lifetime = rclcpp::Duration::from_seconds(0.5);
    line_strip.scale.x = 0.1;
    geometry_msgs::msg::Point point;
    point.x = supervoxel->centroid_.x;
    point.y = supervoxel->centroid_.y;
    point.z = supervoxel->centroid_.z;
    std_msgs::msg::ColorRGBA yellow_color;
    yellow_color.r = 1.0;
    yellow_color.g = 1.0;
    yellow_color.a = 0.4;
    line_strip.points.push_back(point);
    line_strip.colors.push_back(yellow_color);

    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "supervoxel_markers_ns";
    sphere.id = index + 10000;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.lifetime = rclcpp::Duration::from_seconds(3.0);
    sphere.pose.position = point;
    sphere.scale.x = 0.3;
    sphere.scale.y = 0.3;
    sphere.scale.z = 0.3;
    sphere.color.a = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 1.0;

    for (auto adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
         adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr) {
      auto neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);

      geometry_msgs::msg::Point n_point;
      n_point.x = neighbor_supervoxel->centroid_.x;
      n_point.y = neighbor_supervoxel->centroid_.y;
      n_point.z = neighbor_supervoxel->centroid_.z;
      line_strip.points.push_back(n_point);
      line_strip.colors.push_back(yellow_color);
    }
    // Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
    index++;

    marker_array.markers.push_back(sphere);
    marker_array.markers.push_back(line_strip);
  }
}

geometry_msgs::msg::PoseStamped getLinearInterpolatedPose(const geometry_msgs::msg::PoseStamped a, const geometry_msgs::msg::PoseStamped b)
{
  geometry_msgs::msg::PoseStamped linear_interpolated_pose;
  linear_interpolated_pose.pose.position.x = (a.pose.position.x + b.pose.position.x) / 2.0;
  linear_interpolated_pose.pose.position.y = (a.pose.position.y + b.pose.position.y) / 2.0;
  linear_interpolated_pose.pose.position.z = (a.pose.position.z + b.pose.position.z) / 2.0;
  return linear_interpolated_pose;
}

void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> & path,
  const geometry_msgs::msg::Vector3 & marker_scale,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & plan_publisher,
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & nav_path_publisher,
  const bool & publish_segment_ids, const std::string & robot_mesh_path)
{
  // Clear All previous markers
  visualization_msgs::msg::MarkerArray clear_markers;
  visualization_msgs::msg::Marker clear_path;
  clear_path.id = 0;
  clear_path.ns = "path";
  clear_path.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_markers.markers.push_back(clear_path);
  plan_publisher->publish(clear_markers);

  visualization_msgs::msg::MarkerArray marker_array;
  nav_msgs::msg::Path nav_msgs_path;

  auto path_idx = 0;
  for (auto && i : path) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "path";
    marker.id = path_idx;
    if (!robot_mesh_path.empty()) {
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.mesh_resource = robot_mesh_path;
    }
    else {
      marker.type = visualization_msgs::msg::Marker::CUBE;
    }
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.text = std::to_string(path_idx);
    marker.pose = i.pose;
    marker.scale = marker_scale;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker_array.markers.push_back(marker);

    double void_s, yaw;
    vox_nav_utilities::getRPYfromMsgQuaternion(i.pose.orientation, void_s, void_s, yaw);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = rclcpp::Clock().now();
    text.ns = "text";
    text.id = path_idx;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.lifetime = rclcpp::Duration::from_seconds(0);
    std::string dis(std::to_string(path_idx) + "," + std::to_string(yaw));
    if (publish_segment_ids) {
      text.text = dis;
    }
    text.pose = i.pose;
    text.pose.position.z += 0.5;
    text.scale.x = 0.3;
    text.scale.y = 0.3;
    text.scale.z = 0.3;
    text.color.a = 1.0;
    text.color.g = 1.0;
    text.color.r = 1.0;
    path_idx++;
    marker_array.markers.push_back(text);

    nav_msgs_path.poses.push_back(i);
  }
  plan_publisher->publish(marker_array);

  nav_msgs_path.header.frame_id = "map";
  nav_msgs_path.header.stamp = rclcpp::Clock().now();
  nav_path_publisher->publish(nav_msgs_path);
}
}  // namespace vox_nav_utilities
