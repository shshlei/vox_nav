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

#include "vox_nav_planning/planner_server.hpp"
#include <vox_nav_utilities/planner_helpers.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>

using namespace std::chrono_literals;

namespace vox_nav_planning
{
PlannerServer::PlannerServer()
: Node("vox_nav_planning_server_rclcpp_node"),
  pc_loader_("vox_nav_planning", "vox_nav_planning::PlannerCore")
{
  // Declare this node's parameters
  planner_type_ = declare_parameter("planner_type", planner_type_);
  robot_mesh_path_ = declare_parameter("robot_mesh_path", "");
  publish_segment_ids_ = declare_parameter("publish_segment_ids", true);

  declare_parameter("planner_name", "PRMStar");
  declare_parameter("planner_timeout", 5.0);
  declare_parameter("interpolation_parameter", 50);
  declare_parameter("robot_body_dimens.x", 1.0);
  declare_parameter("robot_body_dimens.y", 0.8);
  declare_parameter("robot_body_dimens.z", 0.6);

  try {
    planner_ = pc_loader_.createSharedInstance(planner_type_);
    planner_->initialize(this);
    RCLCPP_INFO(get_logger(), "Created planner plugin of type %s", planner_type_.c_str());
  }
  catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create planner. Exception: %s",
      ex.what());
  }

  // Initialize pubs & subs
  plan_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/planning/plan", 1);
  nav_msgs_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vox_nav/planning/nav_msgs_path", rclcpp::SystemDefaultsQoS());

  this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PlannerServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&PlannerServer::handle_accepted, this, std::placeholders::_1));

  this->action_server_2_ = rclcpp_action::create_server<ComputeStartToGoal>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "compute_start_to_goal",
    std::bind(&PlannerServer::handle_goal_2, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PlannerServer::handle_cancel_2, this, std::placeholders::_1),
    std::bind(&PlannerServer::handle_accepted_2, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

PlannerServer::~PlannerServer()
{
  action_server_.reset();
  action_server_2_.reset();
  plan_publisher_.reset();
  nav_msgs_path_pub_.reset();
}

// Current Start To Goal
rclcpp_action::GoalResponse PlannerServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputePathToPose::Goal> /*goal*/)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request in order to compute a path to pose");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerServer::handle_cancel(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerServer::handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PlannerServer::computePlan, this, std::placeholders::_1), goal_handle}.detach();
}

void PlannerServer::computePlan(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "Received a planning request to (%.3f, %.3f)", goal->pose.pose.position.x, goal->pose.pose.position.y);
  auto feedback = std::make_shared<ComputePathToPose::Feedback>();
  auto result = std::make_shared<ComputePathToPose::Result>();

  geometry_msgs::msg::PoseStamped start_pose, goal_pose = goal->pose;
  vox_nav_utilities::getCurrentPose(start_pose, *tf_buffer_, "map", "base_link", 0.1);

  auto start_time = steady_clock_.now();
  result->path.poses = getPlan(start_pose, goal_pose);
  auto elapsed_time = steady_clock_.now() - start_time;
  result->path.header.frame_id = "map";
  if (result->path.poses.size() == 0) {
    RCLCPP_WARN(get_logger(), "Planning algorithm failed to generate a valid path");
    return;
  }

  // Check if there is a cancel request
  if (goal_handle->is_canceling()) {
    result->path.poses = std::vector<geometry_msgs::msg::PoseStamped>();
    goal_handle->canceled(result);
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    return;
  }

  // Update sequence
  feedback->elapsed_time = elapsed_time;
  goal_handle->publish_feedback(feedback);

  // Check if goal is done
  if (rclcpp::ok()) {
    result->planning_time = elapsed_time;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    // Publish the plan for visualization purposes
    geometry_msgs::msg::Vector3 scale;
    scale.x = get_parameter("robot_body_dimens.x").as_double();
    scale.y = get_parameter("robot_body_dimens.y").as_double();
    scale.z = get_parameter("robot_body_dimens.z").as_double();
    vox_nav_utilities::publishPlan(result->path.poses, scale, plan_publisher_, nav_msgs_path_pub_);
  }
}

// Start To Goal
rclcpp_action::GoalResponse PlannerServer::handle_goal_2(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputeStartToGoal::Goal> /*goal*/)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request in order to compute a start to goal");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerServer::handle_cancel_2(const std::shared_ptr<GoalHandleComputeStartToGoal> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerServer::handle_accepted_2(const std::shared_ptr<GoalHandleComputeStartToGoal> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PlannerServer::computePlan_2, this, std::placeholders::_1), goal_handle}.detach();
}

void PlannerServer::computePlan_2(const std::shared_ptr<GoalHandleComputeStartToGoal> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "Received a planning request to (%.3f, %.3f)", goal->goal.pose.position.x, goal->goal.pose.position.y);
  auto feedback = std::make_shared<ComputeStartToGoal::Feedback>();
  auto result = std::make_shared<ComputeStartToGoal::Result>();

  geometry_msgs::msg::PoseStamped start_pose = goal->start, goal_pose = goal->goal;
  auto start_time = steady_clock_.now();
  result->path.poses = getPlan(start_pose, goal_pose);
  auto elapsed_time = steady_clock_.now() - start_time;
  result->path.header.frame_id = "map";
  if (result->path.poses.size() == 0) {
    RCLCPP_WARN(get_logger(), "Planning algorithm failed to generate a valid path");
    return;
  }

  // Check if there is a cancel request
  if (goal_handle->is_canceling()) {
    result->path.poses = std::vector<geometry_msgs::msg::PoseStamped>();
    goal_handle->canceled(result);
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    return;
  }

  // Update sequence
  feedback->elapsed_time = elapsed_time;
  goal_handle->publish_feedback(feedback);

  // Check if goal is done
  if (rclcpp::ok()) {
    result->planning_time = elapsed_time;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    // Publish the plan for visualization purposes
    geometry_msgs::msg::Vector3 scale;
    scale.x = get_parameter("robot_body_dimens.x").as_double();
    scale.y = get_parameter("robot_body_dimens.y").as_double();
    scale.z = get_parameter("robot_body_dimens.z").as_double();
    vox_nav_utilities::publishPlan(result->path.poses, scale, plan_publisher_, nav_msgs_path_pub_);
  }
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerServer::getPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return planner_->createPlan(start, goal);
}

}  // namespace vox_nav_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_planning::PlannerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
