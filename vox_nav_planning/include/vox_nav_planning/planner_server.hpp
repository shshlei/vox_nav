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

#ifndef VOX_NAV_PLANNING__PLANNER_SERVER_HPP_
#define VOX_NAV_PLANNING__PLANNER_SERVER_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include <vox_nav_msgs/action/compute_path_to_pose.hpp>
#include <vox_nav_msgs/action/compute_start_to_goal.hpp>

#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace vox_nav_planning
{
class PlannerServer : public rclcpp::Node
{
public:
  using ComputePathToPose = vox_nav_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;
  using ComputeStartToGoal = vox_nav_msgs::action::ComputeStartToGoal;
  using GoalHandleComputeStartToGoal = rclcpp_action::ServerGoalHandle<ComputeStartToGoal>;

  PlannerServer();

  virtual ~PlannerServer();

  // Current Pose To Goal
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputePathToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  // Start To Goal
  rclcpp_action::GoalResponse handle_goal_2(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputeStartToGoal::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel_2(const std::shared_ptr<GoalHandleComputeStartToGoal> goal_handle);

  void handle_accepted_2(const std::shared_ptr<GoalHandleComputeStartToGoal> goal_handle);

protected:
  std::vector<geometry_msgs::msg::PoseStamped> getPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);

  // Our action server implements the ComputePathToPose action
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;

  void computePlan(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  rclcpp_action::Server<ComputeStartToGoal>::SharedPtr action_server_2_;

  void computePlan_2(const std::shared_ptr<GoalHandleComputeStartToGoal> goal_handle);

  // Planner
  vox_nav_planning::PlannerCore::Ptr planner_;

  pluginlib::ClassLoader<vox_nav_planning::PlannerCore> pc_loader_;

  std::string planner_type_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // tf buffer to get transfroms
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // tf listner for tf transforms
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers for the path
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_publisher_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_msgs_path_pub_;

  // obot mesh path, if there is one
  std::string robot_mesh_path_;

  bool publish_segment_ids_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLANNER_SERVER_HPP_
