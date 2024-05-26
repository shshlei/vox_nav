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

#ifndef VOX_NAV_PLANNING__BOX2D_PLANNER_CORE_HPP_
#define VOX_NAV_PLANNING__BOX2D_PLANNER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_2d_msgs/msg/path2_d.hpp>
#include <nav_2d_msgs/msg/pose2_d_stamped.hpp>

#include <rviz_visualization_tools/rviz_visualization_tools.h>

// OMPL
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

// STL
#include <memory>
#include <string>
#include <vector>

namespace vox_nav_planning
{
class Box2dPlannerCore
{
public:
  using Ptr = typename std::shared_ptr<Box2dPlannerCore>;

  Box2dPlannerCore() = default;

  virtual ~Box2dPlannerCore() = default;

  virtual void initialize(rclcpp::Node * parent);

  virtual nav_2d_msgs::msg::Path2D createPlan(const nav_2d_msgs::msg::Pose2DStamped & start, const nav_2d_msgs::msg::Pose2DStamped & goal);

protected:

  // curve radius for reeds and dubins only
  ompl::base::StateSpacePtr state_space_;

  ompl::geometric::SimpleSetupPtr simple_setup_;

  // max time the planner can spend before coming up with a solution
  double planner_timeout_;

  bool do_simplification_;

  // related to density of created path
  int interpolation_parameter_;

private:

  rclcpp::Logger logger_{rclcpp::get_logger("box2d_planner")};

private:

  // Publisher Planner Nodes
  bool pub_nodes_;

  // For visualizing things in rviz
  rvt::RvizVisualizationToolsPtr visual_tools_;

private:

  std::vector<double> pose_x_;

  std::vector<double> pose_y_;

  std::vector<double> pose_yaw_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__BOX2D_PLANNER_CORE_HPP_
