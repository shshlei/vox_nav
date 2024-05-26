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

#include "vox_nav_planning/box2d_planner_client.hpp"
#include <nav_2d_msgs/msg/pose2_d_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <functional>

namespace vox_nav_planning
{
Box2dPlannerClient::Box2dPlannerClient()
: Node("vox_nav_planning_box2d_client_rclcpp_node")
{
  this->action_client_ = rclcpp_action::create_client<Compute2DStartToGoal>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "box2d_compute_start_to_goal");

  while (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the compute_2d_start_to_goal action. Exiting");
      return;
    }
    RCLCPP_INFO(get_logger(), "compute_2d_start_to_goal action not available, waiting and trying again");
  }

  double start_x = declare_parameter("start_x", 0.0);
  double start_y = declare_parameter("start_y", 0.0);
  double start_yaw = declare_parameter("start_yaw", 0.0);

  double goal_x = declare_parameter("goal_x", 0.0); 
  double goal_y = declare_parameter("goal_y", 0.0); 
  double goal_yaw = declare_parameter("goal_yaw", 0.0); 

  nav_2d_msgs::msg::Pose2DStamped start_pose, goal_pose;
  start_pose.header.frame_id = "map";
  start_pose.header.stamp = rclcpp::Clock().now();
  start_pose.pose.x = start_x;
  start_pose.pose.y = start_y;
  start_pose.pose.theta = start_yaw;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = rclcpp::Clock().now();
  goal_pose.pose.x = goal_x;
  goal_pose.pose.y = goal_y;
  goal_pose.pose.theta = goal_yaw;

  auto goalMsg = Compute2DStartToGoal::Goal();
  goalMsg.start = start_pose;
  goalMsg.goal = goal_pose;
  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Compute2DStartToGoal>::SendGoalOptions();
  using namespace std::placeholders;
  send_goal_options.goal_response_callback = std::bind(&Box2dPlannerClient::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback = std::bind(&Box2dPlannerClient::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&Box2dPlannerClient::resultCallback, this, _1);
  auto goal_handle_future = this->action_client_->async_send_goal(goalMsg, send_goal_options);
}

Box2dPlannerClient::~Box2dPlannerClient()
{
  action_client_.reset();
}

void Box2dPlannerClient::goalResponseCallback(const GoalHandleCompute2DStartToGoal::SharedPtr & goalHandle)
{
  if (!goalHandle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  }
  else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Box2dPlannerClient::feedbackCallback(GoalHandleCompute2DStartToGoal::SharedPtr, const std::shared_ptr<const Compute2DStartToGoal::Feedback> /*feedback*/)
{
}

void Box2dPlannerClient::resultCallback(const GoalHandleCompute2DStartToGoal::WrappedResult & result)
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
  rclcpp::shutdown();
}
}  // namespace vox_nav_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_planning::Box2dPlannerClient>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
