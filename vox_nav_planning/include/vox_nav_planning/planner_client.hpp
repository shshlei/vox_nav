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

#ifndef VOX_NAV_PLANNING__PLANNER_CLIENT_HPP_
#define VOX_NAV_PLANNING__PLANNER_CLIENT_HPP_

#include <vox_nav_msgs/action/compute_start_to_goal.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>

namespace vox_nav_planning
{
class PlannerClient : public rclcpp::Node
{
public:
  using ComputeStartToGoal = vox_nav_msgs::action::ComputeStartToGoal;
  using GoalHandleComputeStartToGoal = rclcpp_action::ClientGoalHandle<ComputeStartToGoal>;

  PlannerClient();

  virtual ~PlannerClient();

protected:
  rclcpp_action::Client<ComputeStartToGoal>::SharedPtr action_client_;

  void goalResponseCallback(const GoalHandleComputeStartToGoal::SharedPtr & goalHandle);

  void feedbackCallback(GoalHandleComputeStartToGoal::SharedPtr, const std::shared_ptr<const ComputeStartToGoal::Feedback> feedback);

  void resultCallback(const GoalHandleComputeStartToGoal::WrappedResult & result);
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLANNER_CLIENT_HPP_
