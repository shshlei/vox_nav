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

#include "vox_nav_planning/box2d_planner_server.hpp"

namespace vox_nav_planning
{
Box2dPlannerServer::Box2dPlannerServer()
: Node("vox_nav_planning_box2d_server_rclcpp_node")
{
  planner_ = std::make_shared<vox_nav_planning::Box2dPlannerCore>();
  planner_->initialize(this);

  this->action_server_ = rclcpp_action::create_server<Compute2DStartToGoal>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "box2d_compute_start_to_goal",
    std::bind(&Box2dPlannerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Box2dPlannerServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&Box2dPlannerServer::handle_accepted, this, std::placeholders::_1));

  this->action_client_ = rclcpp_action::create_client<Compute2DStartToGoalTrajectory>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "box2d_compute_start_to_goal_trajectory");
  while (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the compute_2d_start_to_goal_trajectory action. Exiting");
      return;
    }
    RCLCPP_INFO(get_logger(), "compute_2d_start_to_goal_trajectory action not available, waiting and trying again");
  }
}

Box2dPlannerServer::~Box2dPlannerServer()
{
  action_server_.reset();
  action_client_.reset();
}

// Path Server
// Start To Goal
rclcpp_action::GoalResponse Box2dPlannerServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Compute2DStartToGoal::Goal> /*goal*/)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request in order to compute a start to goal");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Box2dPlannerServer::handle_cancel(const std::shared_ptr<GoalHandleCompute2DStartToGoal> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Box2dPlannerServer::handle_accepted(const std::shared_ptr<GoalHandleCompute2DStartToGoal> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Box2dPlannerServer::computePlan, this, std::placeholders::_1), goal_handle}.detach();
}

void Box2dPlannerServer::computePlan(const std::shared_ptr<GoalHandleCompute2DStartToGoal> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "Received a planning request to (%.3f, %.3f, %.3f)", goal->goal.pose.x, goal->goal.pose.y, goal->goal.pose.theta);
  auto feedback = std::make_shared<Compute2DStartToGoal::Feedback>();
  auto result = std::make_shared<Compute2DStartToGoal::Result>();

  auto start_time = steady_clock_.now();
  result->path = getPlan(goal->start, goal->goal);
  auto elapsed_time = steady_clock_.now() - start_time;
  if (result->path.poses.size() == 0) {
    RCLCPP_WARN(get_logger(), "Planning algorithm failed to generate a valid path");
    return;
  }

  // Check if there is a cancel request
  if (goal_handle->is_canceling()) {
    result->path = nav_2d_msgs::msg::Path2D();
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

    auto goalMsg = Compute2DStartToGoalTrajectory::Goal();
    goalMsg.path = result->path;
    RCLCPP_INFO(this->get_logger(), "Sending trajectory goal");

    auto send_goal_options = rclcpp_action::Client<Compute2DStartToGoalTrajectory>::SendGoalOptions();
    using namespace std::placeholders;
    send_goal_options.goal_response_callback = std::bind(&Box2dPlannerServer::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&Box2dPlannerServer::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Box2dPlannerServer::resultCallback, this, _1);
    auto goal_handle_future = this->action_client_->async_send_goal(goalMsg, send_goal_options);
  }
}

nav_2d_msgs::msg::Path2D Box2dPlannerServer::getPlan(const nav_2d_msgs::msg::Pose2DStamped & start, const nav_2d_msgs::msg::Pose2DStamped & goal)
{
  return planner_->createPlan(start, goal);
}

// Trajectory Client
void Box2dPlannerServer::goalResponseCallback(const GoalHandleCompute2DStartToGoalTrajectory::SharedPtr & goalHandle)
{
  if (!goalHandle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  }
  else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Box2dPlannerServer::feedbackCallback(GoalHandleCompute2DStartToGoalTrajectory::SharedPtr, const std::shared_ptr<const Compute2DStartToGoalTrajectory::Feedback> /*feedback*/)
{
}

void Box2dPlannerServer::resultCallback(const GoalHandleCompute2DStartToGoalTrajectory::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
  RCLCPP_INFO(this->get_logger(), "Result received");
}
}  // namespace vox_nav_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_planning::Box2dPlannerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
