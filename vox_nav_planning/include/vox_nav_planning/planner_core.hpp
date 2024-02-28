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

#ifndef VOX_NAV_PLANNING__PLANNER_CORE_HPP_
#define VOX_NAV_PLANNING__PLANNER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

// OMPL
#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>

// STL
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace vox_nav_planning
{
class PlannerCore
{
public:
  using Ptr = typename std::shared_ptr<PlannerCore>;

  PlannerCore() = default;

  virtual ~PlannerCore() = default;

  virtual void initialize(rclcpp::Node * parent) = 0;

  virtual std::vector<geometry_msgs::msg::PoseStamped> createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) = 0;

  virtual bool isStateValid(const ompl::base::State * state) = 0;

  virtual void setupMap() = 0;

protected:
  rclcpp::Node::SharedPtr get_map_client_node_;

  ompl::geometric::SimpleSetupPtr simple_setup_;

  // to ensure safety when accessing global var curr_frame_
  std::mutex global_mutex_;

  // the topic to subscribe in order capture a frame
  std::string planner_name_;

  // related to density of created path
  int interpolation_parameter_;

  // max time the planner can spend before coming up with a solution
  double planner_timeout_;

  volatile bool is_map_ready_;
};
}  // namespace vox_nav_planning
#endif  // VOX_NAV_PLANNING__PLANNER_CORE_HPP_
