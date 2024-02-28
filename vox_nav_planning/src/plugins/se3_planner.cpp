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

#include "vox_nav_planning/plugins/se3_planner.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <vox_nav_utilities/planner_helpers.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>

#include <fcl/geometry/octree/octree.h>
#include <fcl/math/constants.h>
#include <fcl/narrowphase/collision.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <memory>
#include <string>
#include <vector>

namespace vox_nav_planning
{

void SE3Planner::initialize(rclcpp::Node * parent)
{
  parent->get_parameter("planner_name", planner_name_);
  parent->get_parameter("planner_timeout", planner_timeout_);
  parent->get_parameter("interpolation_parameter", interpolation_parameter_);
  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

  se3_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(3);
  double minx = parent->declare_parameter("state_space_boundries.minx", -50.0);
  double miny = parent->declare_parameter("state_space_boundries.miny", -10.0);
  double minz = parent->declare_parameter("state_space_boundries.minz", -10.0);
  double maxx = parent->declare_parameter("state_space_boundries.maxx", 50.0);
  double maxy = parent->declare_parameter("state_space_boundries.maxy", 10.0);
  double maxz = parent->declare_parameter("state_space_boundries.maxz", 10.0);
  se3_bounds_->setLow(0, minx);
  se3_bounds_->setLow(1, miny);
  se3_bounds_->setLow(2, minz);
  se3_bounds_->setHigh(0, maxx);
  se3_bounds_->setHigh(1, maxy);
  se3_bounds_->setHigh(2, maxz);

  state_space_ = std::make_shared<ompl::base::SE3StateSpace>();
  state_space_->as<ompl::base::SE3StateSpace>()->setBounds(*se3_bounds_);
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);
  // objective is to minimize the planned path
  ompl::base::OptimizationObjectivePtr objective(new ompl::base::PathLengthOptimizationObjective(simple_setup_->getSpaceInformation()));
  simple_setup_->setOptimizationObjective(objective);
  simple_setup_->setStateValidityChecker(std::bind(&SE3Planner::isStateValid, this, std::placeholders::_1));

  typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(new fcl::Box<float>(parent->get_parameter("robot_body_dimens.x").as_double(),
    parent->get_parameter("robot_body_dimens.y").as_double(),
    parent->get_parameter("robot_body_dimens.z").as_double()));

  fcl::CollisionObjectf robot_body_box_object(robot_body_box, fcl::Transform3f());
  robot_collision_object_ = std::make_shared<fcl::CollisionObjectf>(robot_body_box_object);

  // service hooks for robot localization fromll service
  get_map_client_node_ = std::make_shared<rclcpp::Node>("get_traversability_map_client_node");
  get_traversability_map_client_ = get_map_client_node_->create_client<vox_nav_msgs::srv::GetTraversabilityMap>("get_traversability_map");
  is_map_ready_ = false;
  setupMap();
}

std::vector<geometry_msgs::msg::PoseStamped> SE3Planner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  if (!is_map_ready_) {
    RCLCPP_WARN(logger_, "A valid map has not been receievd yet, Try later again.");
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  start_ = start;
  goal_ = goal;

  ompl::base::ScopedState<ompl::base::SE3StateSpace> se3_start(state_space_), se3_goal(state_space_);
  se3_start->setXYZ(start.pose.position.x, start.pose.position.y, start.pose.position.z);
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->x = start.pose.orientation.x;
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->y = start.pose.orientation.y;
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->z = start.pose.orientation.z;
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->w = start.pose.orientation.w;

  se3_goal->setXYZ(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->x = goal.pose.orientation.x;
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->y = goal.pose.orientation.y;
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->z = goal.pose.orientation.z;
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->w = goal.pose.orientation.w;
  simple_setup_->setStartAndGoalStates(se3_start, se3_goal);

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  vox_nav_utilities::initializeSelectedPlanner(planner, planner_name_, simple_setup_->getSpaceInformation(), logger_);

  simple_setup_->setPlanner(planner);
  simple_setup_->setup();
  // print the settings for this space
  // simple_setup_->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = simple_setup_->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    ompl::geometric::PathGeometric solution_path = simple_setup_->getSolutionPath();
    solution_path.interpolate(interpolation_parameter_);
    for (std::size_t path_idx = 0; path_idx < solution_path.getStateCount(); path_idx++) {
      // cast the abstract state type to the type we expect
      const ompl::base::SE3StateSpace::StateType * se3_state = solution_path.getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = start.header.frame_id;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = se3_state->getX();
      pose.pose.position.y = se3_state->getY();
      pose.pose.position.z = se3_state->getZ();
      pose.pose.orientation.x = se3_state->rotation().x;
      pose.pose.orientation.y = se3_state->rotation().y;
      pose.pose.orientation.z = se3_state->rotation().z;
      pose.pose.orientation.w = se3_state->rotation().w;
      plan_poses.push_back(pose);
    }
    RCLCPP_INFO(logger_, "Found A plan with %ld poses", plan_poses.size());
  }
  else {
    RCLCPP_WARN(logger_, "No solution for requested path planning !");
  }
  return plan_poses;
}

bool SE3Planner::isStateValid(const ompl::base::State * state)
{
  // cast the abstract state type to the type we expect
  const ompl::base::SE3StateSpace::StateType * se3_state = state->as<ompl::base::SE3StateSpace::StateType>();
  // check validity of state Fdefined by pos & rot
  fcl::Vector3f translation(se3_state->getX(), se3_state->getY(), se3_state->getZ());
  fcl::Quaternionf rotation(se3_state->rotation().x, se3_state->rotation().y, se3_state->rotation().z, se3_state->rotation().w);
  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequestf requestType(1, false, 1, false);
  fcl::CollisionResultf collisionResult;
  fcl::collide<float>(robot_collision_object_.get(), original_octomap_collision_object_.get(), requestType, collisionResult);
  return !collisionResult.isCollision();
}

void SE3Planner::setupMap()
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);

  while (!is_map_ready_ && rclcpp::ok()) {
    auto request = std::make_shared<vox_nav_msgs::srv::GetTraversabilityMap::Request>();

    while (!get_traversability_map_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger_, "Interrupted while waiting for the get_traversability_map service. Exiting");
        return;
      }
      RCLCPP_INFO(logger_, "get_traversability_map service not available, waiting and trying again");
    }

    auto result_future = get_traversability_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_map_client_node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "/get_traversability_map service call failed");
    }
    auto response = result_future.get();

    if (response->is_valid) {
      is_map_ready_ = true;
    }
    else {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(logger_, "Waiting for GetTraversabilityMap service to provide correct maps.");
      continue;
    }

    auto original_octomap_octree = dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(response->original_octomap));
    std::shared_ptr<octomap::OcTree> original_octomap_octree_ = std::make_shared<octomap::OcTree>(*original_octomap_octree);
    delete original_octomap_octree;

    auto original_octomap_fcl_octree = std::make_shared<fcl::OcTreef>(original_octomap_octree_);
    original_octomap_collision_object_ = std::make_shared<fcl::CollisionObjectf>(std::shared_ptr<fcl::CollisionGeometryf>(original_octomap_fcl_octree));

    RCLCPP_INFO(logger_,
      "Recieved a valid Octomap with %ld nodes, A FCL collision tree will be created from this "
      "octomap for state validity (aka collision check)",
      original_octomap_octree_->size());
  }
}
}  // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::SE3Planner, vox_nav_planning::PlannerCore)
