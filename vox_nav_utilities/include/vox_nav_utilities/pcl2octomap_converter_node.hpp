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

#ifndef VOX_NAV_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_
#define VOX_NAV_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_

#include "vox_nav_utilities/tf_helpers.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap/ColorOcTree.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <string>

namespace vox_nav_utilities
{
class PCL2OctomapConverter : public rclcpp::Node
{
public:
  PCL2OctomapConverter();

  virtual ~PCL2OctomapConverter() = default;

  void calcThresholdedNodes(const octomap::ColorOcTree & tree, unsigned int & num_thresholded, unsigned int & num_other);

  void outputStatistics(const octomap::ColorOcTree & tree);

  void processConversion();

  void timerCallback();

private:
  // Optional rigid body transform to apply o the cloud, if cloud
  // is depth camera frames we need to pull cloud back to conventional ROS frames
  vox_nav_utilities::RigidBodyTransformation pointloud_transform_matrix_;

  std::string input_pcd_filename_;

  std::string output_binary_octomap_filename_;

  double octomap_voxelsize_;

  // optional point cloud transformfrom yaml file
  double downsample_voxel_size_;

  int remove_outlier_mean_K_;

  double remove_outlier_stddev_threshold_;

  double remove_outlier_radius_search_;

  int remove_outlier_min_neighbors_in_radius_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_;

  // Used to creted a periodic callback function IOT publish transfrom/octomap/cloud etc.
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr octomap_markers_publisher_;

  visualization_msgs::msg::MarkerArray octomap_markers_;
};

}  // namespace vox_nav_utilities
#endif  // VOX_NAV_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_
