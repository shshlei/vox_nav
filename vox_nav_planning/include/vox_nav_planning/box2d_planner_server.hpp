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

#ifndef VOX_NAV_PLANNING__BOX2D_PLANNER_SERVER_HPP_
#define VOX_NAV_PLANNING__BOX2D_PLANNER_SERVER_HPP_

#include "vox_nav_planning/box2d_planner_core.hpp"
#include <vox_nav_msgs/action/compute2_d_start_to_goal.hpp>
#include <vox_nav_msgs/action/compute2_d_start_to_goal_trajectory.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_2d_msgs/msg/path2_d.hpp>
#include <nav_2d_msgs/msg/pose2_d_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace vox_nav_planning
{
class Box2dPlannerServer : public rclcpp::Node
{
public:
  using Compute2DStartToGoal = vox_nav_msgs::action::Compute2DStartToGoal;
  using GoalHandleCompute2DStartToGoal = rclcpp_action::ServerGoalHandle<Compute2DStartToGoal>;

  using Compute2DStartToGoalTrajectory = vox_nav_msgs::action::Compute2DStartToGoalTrajectory;
  using GoalHandleCompute2DStartToGoalTrajectory = rclcpp_action::ClientGoalHandle<Compute2DStartToGoalTrajectory>;

  Box2dPlannerServer();

  virtual ~Box2dPlannerServer();

  // Start To Goal
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Compute2DStartToGoal::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCompute2DStartToGoal> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleCompute2DStartToGoal> goal_handle);

protected:
  void computePlan(const std::shared_ptr<GoalHandleCompute2DStartToGoal> goal_handle);

  rclcpp_action::Server<Compute2DStartToGoal>::SharedPtr action_server_;

protected:

  nav_2d_msgs::msg::Path2D getPlan(const nav_2d_msgs::msg::Pose2DStamped & start, const nav_2d_msgs::msg::Pose2DStamped & goal);

  // Planner
  vox_nav_planning::Box2dPlannerCore::Ptr planner_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

protected:
  rclcpp_action::Client<Compute2DStartToGoalTrajectory>::SharedPtr action_client_;

  void goalResponseCallback(const GoalHandleCompute2DStartToGoalTrajectory::SharedPtr & goalHandle);

  void feedbackCallback(GoalHandleCompute2DStartToGoalTrajectory::SharedPtr, const std::shared_ptr<const Compute2DStartToGoalTrajectory::Feedback> feedback);

  void resultCallback(const GoalHandleCompute2DStartToGoalTrajectory::WrappedResult & result);
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__BOX2D_PLANNER_SERVER_HPP_
