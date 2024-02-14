// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_UTILITIES__ELEVATION_STATE_SPACE_HPP_
#define VOX_NAV_UTILITIES__ELEVATION_STATE_SPACE_HPP_

// ROS
#include <vox_nav_msgs/srv/get_traversability_map.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
#include <vox_nav_utilities/pcl_helpers.hpp>
#include <vox_nav_utilities/planner_helpers.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// PCL
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// OMPL BASE
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
// OCTOMAP
#include <octomap_msgs/msg/octomap.hpp>

#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
#include <octomap_msgs/conversions.h>
// FCL
#include "fcl/geometry/octree/octree.h"

#include <fcl/config.h>
// STL
#include <iostream>
#include <memory>
#include <string>
#include <vector>
// BOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>

namespace ompl
{
namespace base
{
class OctoCostOptimizationObjective : public ompl::base::StateCostIntegralObjective
{
public:
  OctoCostOptimizationObjective(
    const SpaceInformationPtr & si,
    const std::shared_ptr<octomap::OcTree> & elevated_surfels_octree);

  ~OctoCostOptimizationObjective();

  Cost stateCost(const State * s) const override;

  Cost motionCost(const State * s1, const State * s2) const override;

protected:
  // Octree where the elevated surfesl are stored in
  std::shared_ptr<octomap::OcTree> elevated_surfels_octree_;
  rclcpp::Logger logger_{rclcpp::get_logger("octo_cost_optimization_objective")};
};

class ElevationStateSpace : public CompoundStateSpace
{
public:
  class StateType : public CompoundStateSpace::StateType
  {
  public:
    StateType() = default;

    void setXYZV(double x, double y, double z, double v)
    {
      as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = x;
      as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1] = y;
      as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2] = z;
      as<ompl::base::RealVectorStateSpace::StateType>(1)->values[3] = v;
    }

    void setSO2(double yaw)
    {
      as<SO2StateSpace::StateType>(0)->value = yaw;
    }

    SO2StateSpace::StateType * getSO2()
    {
      return as<SO2StateSpace::StateType>(0);
    }

    ompl::base::RealVectorStateSpace::StateType * getXYZV()
    {
      return as<ompl::base::RealVectorStateSpace::StateType>(1);
    }
  };

  enum SE2StateType
  {
    SE2,
    DUBINS,
    REDDSSHEEP
  };

  ElevationStateSpace(
    const SE2StateType & state_type,
    double turningRadius = 1.0,
    bool isSymmetric = false);

  ~ElevationStateSpace() override = default;

  void setBounds(
    const RealVectorBounds & se2_bounds,
    const RealVectorBounds & z_bounds,
    const RealVectorBounds & v_bounds);

  void enforceBounds(State * state) const override;

  const RealVectorBounds getBounds() const;

  State * allocState() const override;

  void freeState(State * state) const override;

  double distance(
    const State * state1,
    const State * state2) const override;

  void interpolate(
    const State * from,
    const State * to,
    double t,
    State * state) const override;

  void printState(const State * state, std::ostream & out) const override;

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("elevation_state_space")};

  SE2StateType se2_state_type_;
  std::shared_ptr<ompl::base::DubinsStateSpace> dubins_;
  std::shared_ptr<ompl::base::ReedsSheppStateSpace> reeds_sheep_;
  std::shared_ptr<ompl::base::SE2StateSpace> se2_;
  std::shared_ptr<ompl::base::RealVectorStateSpace> real_vector_;
  std::shared_ptr<ompl::base::SO2StateSpace> so2_;

  ompl::base::State * state1_se2_;
  ompl::base::State * state2_se2_;

  ompl::base::State * interpolation_state1_se2_;
  ompl::base::State * interpolation_state2_se2_;
  ompl::base::State * interpolated_state_se2_;

  double rho_;
  bool isSymmetric_;
};

class OctoCellValidStateSampler : public ValidStateSampler
{
public:
  OctoCellValidStateSampler(
    const ompl::base::SpaceInformationPtr & si,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses);

  bool sample(ompl::base::State * state) override;

  bool sampleNear(
    ompl::base::State * state,
    const ompl::base::State * near,
    const double distance) override;

  void updateSearchArea(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("octo_cell_valid_state_sampler")};
  geometry_msgs::msg::PoseArray elevated_surfels_poses_;
  pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels_;
  pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels_;
  std::discrete_distribution<> distrubutions_;
  std::random_device rd_;
  std::mt19937 rng_;

  std::shared_ptr<std::uniform_int_distribution<>> int_distr_;
};

class ElevationStateSpaceProjection : public base::ProjectionEvaluator
{
public:
  ElevationStateSpaceProjection(const ompl::base::StateSpace * space)
  : base::ProjectionEvaluator(space)
  {
  }

  virtual unsigned int getDimension(void) const
  {
    return 2;
  }

  virtual void defaultCellSizes(void)
  {
    cellSizes_.resize(2);
    cellSizes_[0] = 0.25;
    cellSizes_[1] = 0.25;
  }

  virtual void project(const base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const
  {
    const auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
    //const auto * so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
    const auto * xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

    projection(0) = xyzv->values[0];
    projection(1) = xyzv->values[1];
  }
};

}  // namespace base
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__PLUGINS__ELEVATION_STATE_SPACE_HPP_
