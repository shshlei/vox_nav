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

#ifndef VOX_NAV_MAP_SERVER__MAP_MANAGER_HELPERS_HPP_
#define VOX_NAV_MAP_SERVER__MAP_MANAGER_HELPERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace vox_nav_map_server
{
void fillOctomapMarkers(visualization_msgs::msg::MarkerArray::SharedPtr & marker_array,
  const std_msgs::msg::Header::SharedPtr & header,
  const std::shared_ptr<octomap::OcTree> & octree);

// Given a pointcloud, denoise it with use of K- neighbour points and return a pointer to denoised cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoise_segmented_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  const double radius, const double tolerated_divergence_rate, const int min_num_neighbours);

// Traverability is encoded in RGB color channels. R > 0 is  NON traversale
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_non_traversable_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Traverability is encoded in RGB color channels. R > 0 is  NON traversale
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_non_traversable_points(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

// Traverability is encoded in RGB color channels. G > 0 is traversale
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_traversable_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Traverability is encoded in RGB color channels. G > 0 is traversale
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_traversable_points(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

// Center of these cells are basically uniformly sampled cloud.
std::vector<std::pair<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> surfelize_traversability_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pure_traversable_pcl,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr uniformly_sampled_nodes,
  const double radius);

// This function is used o fit a plane model to each cell of traversability cloud.
bool fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const double dist_thes);

// Set the cloud color object. Paints clouds color to given colors.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr set_cloud_color(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::vector<double> colors);

// given plane model, calculate yaw pitch roll from this plane
std::vector<double> rpy_from_plane(const pcl::ModelCoefficients plane_model);

// Average perpendicular distance of outlier points from plane model.
double average_point_deviation_from_plane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::ModelCoefficients plane_model);

// Finds min and max height differnce between edge points.
double max_energy_gap_in_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

}  // namespace vox_nav_map_server

#endif  // VOX_NAV_MAP_SERVER__MAP_MANAGER_HELPERS_HPP_
