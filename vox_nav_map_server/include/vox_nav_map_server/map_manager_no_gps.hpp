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

#include <vox_nav_msgs/srv/get_traversability_map.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <memory>
#include <string>
#include <vector>

namespace vox_nav_map_server
{
class MapManagerNoGPS : public rclcpp::Node
{
public:
  struct PCDPreProcessingParams
  {
    double pcd_map_downsample_voxel_size;

    int remove_outlier_mean_K;

    double remove_outlier_stddev_threshold;

    double remove_outlier_radius_search;

    int remove_outlier_min_neighbors_in_radius;

    bool apply_filters;

    bool use_K_remove_outlier;

    PCDPreProcessingParams()
    : pcd_map_downsample_voxel_size(0.1), remove_outlier_mean_K(10), remove_outlier_stddev_threshold(0.1), remove_outlier_radius_search(0.1), remove_outlier_min_neighbors_in_radius(1), apply_filters(false), use_K_remove_outlier(true)
    {
    }
  };

  struct CostRegressionParams
  {
    double uniform_sample_radius;

    double surfel_radius;

    double max_allowed_tilt;

    double max_allowed_point_deviation;

    double max_allowed_energy_gap;

    double node_elevation_distance;

    double plane_fit_threshold;

    double robot_mass;

    double average_speed;

    double max_color_range;

    std::vector<double> cost_critic_weights;

    CostRegressionParams()
    : uniform_sample_radius(0.2), surfel_radius(0.1), max_allowed_tilt(10), max_allowed_point_deviation(0.1), max_allowed_energy_gap(0.1), node_elevation_distance(1),
    plane_fit_threshold(10), robot_mass(0.1), average_speed(0.1), max_color_range(255.0), cost_critic_weights({0.33, 0.33, 0.33})
    {
    }
  };

  MapManagerNoGPS();

  virtual ~MapManagerNoGPS() = default;

protected:

  void timerCallback();

  void preProcessPCDMap();

  void regressCosts();

  void handleOriginalOctomap();

  void publishMapVisuals();

  void getGetTraversabilityMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap::Request> request,
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap::Response> response);

  std::string pcd_map_filename_;

  // Pointcloud map is stroed here
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_map_pointcloud_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pure_traversable_pointcloud_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pure_non_traversable_pointcloud_;

  pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_pointcloud_;

  // PointCloud Msg
  sensor_msgs::msg::PointCloud2::SharedPtr octomap_pointcloud_msg_;

  sensor_msgs::msg::PointCloud2::SharedPtr traversable_pointcloud_msg_;

  sensor_msgs::msg::PointCloud2::SharedPtr non_traversable_pointcloud_msg_;

  sensor_msgs::msg::PointCloud2::SharedPtr elevated_surfel_pointcloud_msg_;

  // Markers
  visualization_msgs::msg::MarkerArray::SharedPtr original_octomap_markers_msg_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pointloud_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_pointcloud_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_traversable_pointcloud_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevated_surfel_pointcloud_publisher_;

  // Publisher Markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr octomap_markers_publisher_;

  // Octomap Msg
  octomap_msgs::msg::Octomap::SharedPtr original_octomap_msg_;

  octomap_msgs::msg::Octomap::SharedPtr collision_octomap_msg_;

  // Used to call a periodic callback function IOT publish octomap visuals
  rclcpp::TimerBase::SharedPtr timer_;

  // Service to provide Octomap, elevated surfel and elevated surfel poses
  rclcpp::Service<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr get_traversability_map_service_;

  // rclcpp parameters from yaml file: topic name for published octomap as cloud
  std::string octomap_point_cloud_publish_topic_;

  std::string traversable_pointcloud_publish_topic_;

  std::string non_traversable_pointcloud_publish_topic_;

  std::string elevated_surfel_pointcloud_publish_topic_;

  std::string octomap_markers_publish_topic_;

  // rclcpp parameters from yaml file: frame id for map typicall: "map"
  std::string map_frame_id_;

  // rclcpp parameters from yaml file: voxel size for octomap
  double octomap_voxel_size_;

  // rclcpp parameters from yaml file: publish frequncy to publish map and transfroms
  int octomap_publish_frequency_;

  // rclcpp parameters from yaml file: if true, a cloud will be published which represents octomap
  bool publish_octomap_visuals_;

  // Optional rigid body transform to apply to the cloud, if cloud
  // is depth camera frames we need to pull cloud back to conventional ROS frames
  vox_nav_utilities::RigidBodyTransformation pcd_map_transform_matrix_;

  //  see the struct, it is used to keep preprocess params orginzed
  PCDPreProcessingParams preprocess_params_;

  //  see the struct, it is used to keep cost regression params orginzed
  CostRegressionParams cost_params_;

  // hther map has beene configured yet
  volatile bool map_configured_;

  // we need to align static map to map only once, since it is static !
  std::once_flag configure_map_once_;
};
}  // namespace vox_nav_map_server

#endif  // VOX_NAV_MAP_SERVER__MAP_MANAGER_NO_GPS_HPP_
