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

#ifndef VOX_NAV_UTILITIES__PLANNER_BENCHMARKING_HPP_
#define VOX_NAV_UTILITIES__PLANNER_BENCHMARKING_HPP_

// ROS
#include <vox_nav_msgs/srv/get_traversability_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// OMPL BASE
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
// OMPL GEOMETRIC
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
// OMPL TOOL
#include <ompl/tools/benchmark/Benchmark.h>
// OCTOMAP
#include <octomap/octomap.h>
// FCL
#include <fcl/narrowphase/collision_object.h>

// STL
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace vox_nav_utilities
{

struct GroundRobotPose
{
  double x;
  double y;
  double z;
  double yaw;
  GroundRobotPose()
  : x(0.0), y(0.0), z(0.0), yaw(0.0) {}
};

struct SEBounds
{
  double minx;
  double maxx;
  double miny;
  double maxy;
  double minz;
  double maxz;
  double minyaw;
  double maxyaw;
  SEBounds()
  : minx(0.0), maxx(0.0), miny(0.0), maxy(0.0), minz(0.0), maxz(0.0), minyaw(0.0), maxyaw(0.0) {}
};

class PlannerBenchMarking : public rclcpp::Node
{
public:
  volatile bool is_map_ready_;
  /**
   * @brief Construct a new Planner Bench Marking object
   *
   */
  PlannerBenchMarking();

  /**
   * @brief Destroy the Planner Bench Marking object
   *
   */
  ~PlannerBenchMarking();

  /**
   * @brief perfrom actual benchmark and return a sample run
   *
   */
  std::map<int, ompl::geometric::PathGeometric> doBenchMarking();

  /**
   * @brief Callback to subscribe ang get octomap
   *
   * @param octomap
   */
  void setupMap();

  /**
   * @brief
   *
   * @param state
   * @return true
   * @return false
   */
  bool isStateValidSE2(const ompl::base::State * state);

  /**
   * @brief
   *
   * @param state
   * @return true
   * @return false
   */
  bool isStateValidSE3(const ompl::base::State * state);

  /**
   * @brief  make a plan with specified simple setup and planner
   *
   * @param planner
   * @param si
   * @return ompl::geometric::PathGeometric
   */
  ompl::geometric::PathGeometric
  makeAPlan(
    const ompl::base::PlannerPtr & planner,
    ompl::geometric::SimpleSetup & ss);

  /**
   * @brief publish sample plan from bencmarking as marker array into RVIZ
   *
   */
  void publishSamplePlans(
    std::map<int, ompl::geometric::PathGeometric> sample_paths);

  /**
   * @brief Get the Color By Index object
   *
   * @param index
   * @return std_msgs::msg::ColorRGBA
   */
  std_msgs::msg::ColorRGBA getColorByIndex(int index);

  /**
   * @brief
   *
   * @param planner
   * @param selected_planner_name
   * @param si
   */
  void allocatePlannerbyName(
    ompl::base::PlannerPtr & planner,
    const std::string & selected_planner_name,
    const ompl::base::SpaceInformationPtr & si);

  /**
   * @brief Get the Ranged Random object, return a random double in min max
   * range
   *
   * @param min
   * @param max
   * @return double
   */
  double getRangedRandom(double min, double max);

private:
  std::string selected_state_space_;  // se2 ? se3

  SEBounds se_bounds_;                // struct for keeping things clean

  std::shared_ptr<ompl::base::RealVectorBounds> ompl_se_bounds_;

  ompl::base::StateSpacePtr state_space_;

  ompl::base::SpaceInformationPtr si;

  std::vector<std::string> selected_planners_;

  std::string results_output_dir_;

  std::string results_file_regex_;

  double octomap_voxel_size_;

  double planner_timeout_;

  // Only used for REEDS or DUBINS
  double min_turning_radius_;

  double goal_tolerance_;

  double min_euclidean_dist_start_to_goal_;

  int interpolation_parameter_;

  int batch_size_;

  int epochs_;

  int max_memory_;

  bool publish_a_sample_bencmark_;

  std::string sample_bencmark_plans_topic_;

  GroundRobotPose start_;

  GroundRobotPose goal_;

  geometry_msgs::msg::Vector3 robot_body_dimensions_;

  std::shared_ptr<octomap::OcTree> original_octomap_octree_;

  std::shared_ptr<fcl::CollisionObjectf> original_octomap_collision_object_;

  std::shared_ptr<fcl::CollisionObjectf> robot_collision_object_;
  // Publishers for the path

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr start_goal_poses_publisher_;

  rclcpp::Client<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr get_traversability_map_client_;

  rclcpp::Node::SharedPtr get_map_client_node_;

  std::mutex octomap_mutex_;

};
}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__PLANNER_BENCHMARKING_HPP_
