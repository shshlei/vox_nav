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
: Node("vox_nav_map_manager_no_gps_rclcpp_node"), map_configured_(false)
{
  // initialize shared pointers asap
  original_octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  collision_octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  octomap_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  traversable_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  non_traversable_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  elevated_surfel_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  original_octomap_markers_msg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();

  // Declare this node's parameters
  map_frame_id_ = declare_parameter("map_frame_id", "map");
  pcd_map_filename_ = declare_parameter("pcd_map_filename", "");
  octomap_voxel_size_ = declare_parameter("octomap_voxel_size", 0.2);
  octomap_publish_frequency_ = declare_parameter("octomap_publish_frequency", 10);
  publish_octomap_visuals_ = declare_parameter("publish_octomap_visuals", true);
  octomap_point_cloud_publish_topic_ = declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");
  traversable_pointcloud_publish_topic_ = declare_parameter("traversable_pointcloud_publish_topic", "traversable_pointcloud");
  non_traversable_pointcloud_publish_topic_ = declare_parameter("non_traversable_pointcloud_publish_topic", "non_traversable_pointcloud");
  elevated_surfel_pointcloud_publish_topic_ = declare_parameter("elevated_surfel_pointcloud_publish_topic", "elevated_surfel_pointcloud");
  octomap_markers_publish_topic_ = declare_parameter("octomap_markers_publish_topic", "octomap_markers");

  pcd_map_transform_matrix_.translation_.x() = declare_parameter("pcd_map_transform.translation.x", 0.0);
  pcd_map_transform_matrix_.translation_.y() = declare_parameter("pcd_map_transform.translation.y", 0.0);
  pcd_map_transform_matrix_.translation_.z() = declare_parameter("pcd_map_transform.translation.z", 0.0);
  pcd_map_transform_matrix_.rpyIntrinsic_.x() = declare_parameter("pcd_map_transform.rotation.r", 0.0);
  pcd_map_transform_matrix_.rpyIntrinsic_.y() = declare_parameter("pcd_map_transform.rotation.p", 0.0);
  pcd_map_transform_matrix_.rpyIntrinsic_.z() = declare_parameter("pcd_map_transform.rotation.y", 0.0);

  preprocess_params_.apply_filters = declare_parameter("apply_filters", true);
  preprocess_params_.use_K_remove_outlier = declare_parameter("use_K_remove_outlier", true);
  preprocess_params_.pcd_map_downsample_voxel_size = declare_parameter("pcd_map_downsample_voxel_size", 0.1);
  preprocess_params_.remove_outlier_mean_K = declare_parameter("remove_outlier_mean_K", 10);
  preprocess_params_.remove_outlier_stddev_threshold = declare_parameter("remove_outlier_stddev_threshold", 1.0);
  preprocess_params_.remove_outlier_radius_search = declare_parameter("remove_outlier_radius_search", 0.1);
  preprocess_params_.remove_outlier_min_neighbors_in_radius = declare_parameter("remove_outlier_min_neighbors_in_radius", 1);

  cost_params_.uniform_sample_radius = declare_parameter("uniform_sample_radius", 0.2);
  cost_params_.surfel_radius = declare_parameter("surfel_radius", 0.8);
  cost_params_.max_allowed_tilt = declare_parameter("max_allowed_tilt", 40.0);
  cost_params_.max_allowed_point_deviation = declare_parameter("max_allowed_point_deviation", 0.2);
  cost_params_.max_allowed_energy_gap = declare_parameter("max_allowed_energy_gap", 0.2);
  cost_params_.node_elevation_distance = declare_parameter("node_elevation_distance", 0.5);
  cost_params_.plane_fit_threshold = declare_parameter("plane_fit_threshold", 0.2);
  cost_params_.robot_mass = declare_parameter("robot_mass", 0.1);
  cost_params_.average_speed = declare_parameter("average_speed", 1.0);
  cost_params_.cost_critic_weights = declare_parameter("cost_critic_weights", std::vector<double>({0.8, 0.1, 0.1}));

  // service hooks for get maps and surfels
  get_traversability_map_service_ = this->create_service<vox_nav_msgs::srv::GetTraversabilityMap>(
    std::string("get_traversability_map"),
    std::bind(&MapManagerNoGPS::getGetTraversabilityMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  octomap_pointloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    octomap_point_cloud_publish_topic_, rclcpp::SystemDefaultsQoS());

  traversable_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    traversable_pointcloud_publish_topic_, rclcpp::SystemDefaultsQoS());

  non_traversable_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    non_traversable_pointcloud_publish_topic_, rclcpp::SystemDefaultsQoS());

  elevated_surfel_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    elevated_surfel_pointcloud_publish_topic_, rclcpp::SystemDefaultsQoS());

  octomap_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    octomap_markers_publish_topic_, rclcpp::SystemDefaultsQoS());

  pcd_map_pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(pcd_map_filename_.c_str());
  RCLCPP_INFO(this->get_logger(), "Loaded a PCD map with %ld points", pcd_map_pointcloud_->points.size());

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

void MapManagerNoGPS::preProcessPCDMap()
{
  RCLCPP_INFO(get_logger(), "preProcessPCDMap");

  if (preprocess_params_.pcd_map_downsample_voxel_size > 0.0) {
    pcd_map_pointcloud_ = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
      pcd_map_pointcloud_, preprocess_params_.pcd_map_downsample_voxel_size);

    RCLCPP_INFO(this->get_logger(), "PCD Map downsampled, it now has %ld points"
        " adjust the parameters if the map looks off", pcd_map_pointcloud_->points.size());
  }

  if (preprocess_params_.apply_filters) {
    if (preprocess_params_.use_K_remove_outlier) {
      pcd_map_pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
          pcd_map_pointcloud_, preprocess_params_.remove_outlier_mean_K,
          preprocess_params_.remove_outlier_stddev_threshold,
          vox_nav_utilities::OutlierRemovalType::StatisticalOutlierRemoval);
    }
    else {
      pcd_map_pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
          pcd_map_pointcloud_, preprocess_params_.remove_outlier_min_neighbors_in_radius,
          preprocess_params_.remove_outlier_radius_search,
          vox_nav_utilities::OutlierRemovalType::RadiusOutlierRemoval);
    }
    pcd_map_pointcloud_ = vox_nav_utilities::removeNans<pcl::PointXYZRGB>(pcd_map_pointcloud_);
    RCLCPP_INFO(this->get_logger(),
      "Applied a series of noise removal functions"
      " PCD Map downsampled, it now has %ld points",
      pcd_map_pointcloud_->points.size());
  }

  // apply a rigid body transfrom if it was given one
  Eigen::Affine3d bt = vox_nav_utilities::getRigidBodyTransform(pcd_map_transform_matrix_.translation_, pcd_map_transform_matrix_.rpyIntrinsic_);
  auto final_tr = tf2::eigenToTransform(bt);
  pcl_ros::transformPointCloud(*pcd_map_pointcloud_, *pcd_map_pointcloud_, final_tr);

  // Experimental, this assumes we have no prior infromation of
  // segmentation, so mark all points as traversable
  // by painting them green > 0
  pcd_map_pointcloud_ = vox_nav_map_server::set_cloud_color(pcd_map_pointcloud_, std::vector<double>({0.0, 255.0, 0.0}));
}

void MapManagerNoGPS::regressCosts()
{
  RCLCPP_INFO(get_logger(), "regressCosts");

  // seperate traversble points from non-traversable ones
  auto pure_traversable_pcl = vox_nav_map_server::get_traversable_points(pcd_map_pointcloud_);
  auto pure_non_traversable_pcl = vox_nav_map_server::get_non_traversable_points(pcd_map_pointcloud_);

  // uniformly sample nodes on top of traversable cloud
  auto uniformly_sampled_nodes = vox_nav_utilities::uniformlySampleCloud<pcl::PointXYZRGB>(pure_traversable_pcl, cost_params_.uniform_sample_radius);

  // This is basically vector of cloud segments, each segments includes points representing a cell
  // The first element of pair is surfel_center_point while the second is pointcloud itself
  auto surfels = vox_nav_map_server::surfelize_traversability_cloud(pure_traversable_pcl, uniformly_sampled_nodes, cost_params_.surfel_radius);

  // this is acquired by merging all surfels
  pcl::PointCloud<pcl::PointXYZRGB> cost_regressed_cloud;
  // this is acquired by merging only elevated surfel cenroids
  pcl::PointCloud<pcl::PointSurfel> elevated_surfel_pointcloud;

  for (auto && i : surfels) {
    const auto & surfel_center_point = i.first;
    auto surfel_cloud = i.second;

    // fit a plane to this surfel cloud, in order to et its orientation
    pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);
    try {
      vox_nav_map_server::fit_plane_to_cloud(plane_model, surfel_cloud, cost_params_.plane_fit_threshold);
    }
    catch (...) {
      RCLCPP_ERROR(get_logger(),
        "Cannot fit a plane to current surfel points, this may occur if cell size is too small");
      RCLCPP_ERROR(get_logger(), "Current surfel has %ld points, Jumping to next surfel", surfel_cloud->points.size());
      continue;
    }

    // extract rpy from plane equation
    auto rpy = vox_nav_map_server::rpy_from_plane(*plane_model);
    double max_tilt = std::max(std::abs(rpy[0]), std::abs(rpy[1]));

    // any roll or pitch thats higher than max_tilt will make that surfel NON traversable
    if (max_tilt > cost_params_.max_allowed_tilt) {
      surfel_cloud = vox_nav_map_server::set_cloud_color(surfel_cloud, std::vector<double>({255.0, 0, 0}));
    }
    else {
      // extract averge point deviation from surfel cloud this determines the roughness of cloud
      double average_point_deviation = vox_nav_map_server::average_point_deviation_from_plane(surfel_cloud, *plane_model);

      // extract max energy grap from surfel cloud, the higher this , the higher cost
      double max_energy_gap = vox_nav_map_server::max_energy_gap_in_cloud(surfel_cloud);

      // regulate all costs to be less than 1.0
      double slope_cost = std::min(max_tilt / cost_params_.max_allowed_tilt, 1.0) * cost_params_.max_color_range;
      double deviation_of_points_cost = std::min(average_point_deviation / cost_params_.max_allowed_point_deviation, 1.0) * cost_params_.max_color_range;
      double energy_gap_cost = std::min(max_energy_gap / cost_params_.max_allowed_energy_gap, 1.0) * cost_params_.max_color_range;

      double total_cost = cost_params_.cost_critic_weights[0] * slope_cost +
        cost_params_.cost_critic_weights[1] * deviation_of_points_cost +
        cost_params_.cost_critic_weights[2] * energy_gap_cost;

      surfel_cloud = vox_nav_map_server::set_cloud_color(surfel_cloud, std::vector<double>({0.0, cost_params_.max_color_range - total_cost, total_cost}));

      pcl::PointSurfel sp_up;
      sp_up.x = surfel_center_point.x + cost_params_.node_elevation_distance * plane_model->values[0];
      sp_up.y = surfel_center_point.y + cost_params_.node_elevation_distance * plane_model->values[1];
      sp_up.z = surfel_center_point.z + cost_params_.node_elevation_distance * plane_model->values[2];
      sp_up.normal_x = plane_model->values[0];
      sp_up.normal_y = plane_model->values[1];
      sp_up.normal_z = plane_model->values[2];
      sp_up.r = 0.0;
      sp_up.g = cost_params_.max_color_range - total_cost;
      sp_up.b = total_cost;
      elevated_surfel_pointcloud.points.push_back(sp_up);
    }
    cost_regressed_cloud += *surfel_cloud;
  }

  elevated_surfel_pointcloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointSurfel>>(elevated_surfel_pointcloud);

  auto header = std::make_shared<std_msgs::msg::Header>();
  header->frame_id = map_frame_id_;
  header->stamp = this->now();

  cost_regressed_cloud += *pure_non_traversable_pcl;
  *pcd_map_pointcloud_ = cost_regressed_cloud;

  pure_traversable_pointcloud_ = vox_nav_map_server::get_traversable_points(pcd_map_pointcloud_);
  pure_non_traversable_pointcloud_ = vox_nav_map_server::get_non_traversable_points(pcd_map_pointcloud_);
}

void MapManagerNoGPS::handleOriginalOctomap()
{
  RCLCPP_INFO(get_logger(), "handleOriginalOctomap");

  pcl::toROSMsg(*pcd_map_pointcloud_, *octomap_pointcloud_msg_);
  pcl::toROSMsg(*pure_traversable_pointcloud_, *traversable_pointcloud_msg_);
  pcl::toROSMsg(*pure_non_traversable_pointcloud_, *non_traversable_pointcloud_msg_);
  pcl::toROSMsg(*elevated_surfel_pointcloud_, *elevated_surfel_pointcloud_msg_);

  octomap::Pointcloud octocloud, collision_octocloud;
  for (auto && i : pcd_map_pointcloud_->points) {
    octocloud.push_back(octomap::point3d(i.x, i.y, i.z));
  }
  for (auto && i : pure_non_traversable_pointcloud_->points) {
    collision_octocloud.push_back(octomap::point3d(i.x, i.y, i.z));
  }
  auto original_octomap_octree = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  auto collision_octomap_octree = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  original_octomap_octree->insertPointCloud(octocloud, octomap::point3d(0, 0, 0));
  collision_octomap_octree->insertPointCloud(collision_octocloud, octomap::point3d(0, 0, 0));

  for (auto && i : pcd_map_pointcloud_->points) {
    double value = static_cast<double>(i.b / 255.0) - static_cast<double>(i.g / 255.0);
    if (i.r == 255) {
      value = 2.0;
    }
    original_octomap_octree->setNodeValue(i.x, i.y, i.z, std::max(0.0, value));
  }

  for (auto && i : pure_non_traversable_pointcloud_->points) {
    double value = 2.0;
    collision_octomap_octree->setNodeValue(i.x, i.y, i.z, std::max(0.0, value));
  }

  auto header = std::make_shared<std_msgs::msg::Header>();
  header->frame_id = map_frame_id_;
  header->stamp = this->now();
  vox_nav_map_server::fillOctomapMarkers(original_octomap_markers_msg_, header, original_octomap_octree);

  try {
    octomap_msgs::fullMapToMsg<octomap::OcTree>(*original_octomap_octree, *original_octomap_msg_);
    original_octomap_msg_->binary = false;
    original_octomap_msg_->resolution = octomap_voxel_size_;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception while converting original binary octomap %s:", e.what());
  }

  try {
    octomap_msgs::fullMapToMsg<octomap::OcTree>(*collision_octomap_octree, *collision_octomap_msg_);
    collision_octomap_msg_->binary = false;
    collision_octomap_msg_->resolution = octomap_voxel_size_;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception while converting collision binary octomap %s:", e.what());
  }
}

void MapManagerNoGPS::publishMapVisuals()
{
  if (publish_octomap_visuals_) {
    octomap_pointcloud_msg_->header.frame_id = map_frame_id_;
    octomap_pointcloud_msg_->header.stamp = this->now();
    traversable_pointcloud_msg_->header.frame_id = map_frame_id_;
    traversable_pointcloud_msg_->header.stamp = this->now();
    non_traversable_pointcloud_msg_->header.frame_id = map_frame_id_;
    non_traversable_pointcloud_msg_->header.stamp = this->now();
    elevated_surfel_pointcloud_msg_->header.frame_id = map_frame_id_;
    elevated_surfel_pointcloud_msg_->header.stamp = this->now();

    octomap_pointloud_publisher_->publish(*octomap_pointcloud_msg_);
    traversable_pointcloud_publisher_->publish(*traversable_pointcloud_msg_);
    non_traversable_pointcloud_publisher_->publish(*non_traversable_pointcloud_msg_);
    elevated_surfel_pointcloud_publisher_->publish(*elevated_surfel_pointcloud_msg_);
    octomap_markers_publisher_->publish(*original_octomap_markers_msg_);
  }
}

void MapManagerNoGPS::getGetTraversabilityMapCallback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap::Request> /*request*/,
  std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap::Response> response)
{
  if (!map_configured_) {
    RCLCPP_INFO(get_logger(), "Map has not been configured yet,  cannot handle GetTraversabilityMap request");
    response->is_valid = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Map is Cofigured Handling an incoming request");
  response->original_octomap = *original_octomap_msg_;
  response->collision_octomap = *collision_octomap_msg_;
  response->traversable_elevated_cloud = *elevated_surfel_pointcloud_msg_;
  response->traversable_cloud = *traversable_pointcloud_msg_;
  response->is_valid = true;
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
