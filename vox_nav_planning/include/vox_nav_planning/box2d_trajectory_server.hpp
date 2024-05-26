// Copyright (c) 2019 Samsung Research America
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

#ifndef VOX_NAV_PLANNING__BOX2D_TRAJECTORY_SERVER_HPP_
#define VOX_NAV_PLANNING__BOX2D_TRAJECTORY_SERVER_HPP_

#include "vox_nav_planning/differential_driving.hpp"
#include <vox_nav_msgs/action/compute2_d_start_to_goal_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_2d_msgs/msg/path2_d.hpp>
#include <rviz_visualization_tools/rviz_visualization_tools.h>

#include <box2d_collision/b2_bvh_manager.h>

#include <memory>
#include <string>
#include <vector>

namespace vox_nav_planning
{
class Box2dTrajectoryServer : public rclcpp::Node
{
public:
  using Compute2DStartToGoalTrajectory = vox_nav_msgs::action::Compute2DStartToGoalTrajectory;
  using GoalHandleCompute2DStartToGoalTrajectory = rclcpp_action::ServerGoalHandle<Compute2DStartToGoalTrajectory>;

  Box2dTrajectoryServer();

  virtual ~Box2dTrajectoryServer();

  // Start To Goal
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Compute2DStartToGoalTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCompute2DStartToGoalTrajectory> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleCompute2DStartToGoalTrajectory> goal_handle);

protected:
  void computePlan(const std::shared_ptr<GoalHandleCompute2DStartToGoalTrajectory> goal_handle);

  rclcpp_action::Server<Compute2DStartToGoalTrajectory>::SharedPtr action_server_;

private:

  void initialize_problem();

  nav_2d_msgs::msg::Path2D getPlan(const nav_2d_msgs::msg::Path2D & path);

protected:

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

private:

  // For visualizing things in rviz
  rvt::RvizVisualizationToolsPtr visual_tools_;

private:

  int num_iterative_ = 1;

  double active_dist_ = 2.0;

  double vehicle_half_length_ = 0.3;

  double vehicle_half_width_ = 0.3;

  b2RectangleShape * rect_{nullptr};

  std::shared_ptr<b2BVHManager> svc_;

  psopt::ProblemInfo<double>* info_{nullptr};

  DifferentialDriving<double>* problem_{nullptr};
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__BOX2D_TRAJECTORY_SERVER_HPP_
