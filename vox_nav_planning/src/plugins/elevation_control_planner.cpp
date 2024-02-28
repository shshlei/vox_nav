// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_planning/plugins/elevation_control_planner.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <vox_nav_utilities/planner_helpers.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace vox_nav_planning
{

ElevationControlPlanner::ElevationControlPlanner()
{
}

ElevationControlPlanner::~ElevationControlPlanner()
{
}

void ElevationControlPlanner::initialize(rclcpp::Node * parent)
{
  parent->get_parameter("planner_name", planner_name_);
  parent->get_parameter("planner_timeout", planner_timeout_);
  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

  elevated_surfel_cloud_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);

  // declare only planner specific parameters here
  // common parameters are declared in server
  std::string selected_se2_space_name = parent->declare_parameter("se2_space", "REEDS");
  rho_ = parent->declare_parameter(".rho", 1.5);
  double minx = parent->declare_parameter("state_space_boundries.minx", -10.0);
  double miny = parent->declare_parameter("state_space_boundries.miny", -10.0);
  double minz = parent->declare_parameter("state_space_boundries.minz", -10.0);
  double minv = parent->declare_parameter("state_space_boundries.minv", -1.5);
  double maxx = parent->declare_parameter("state_space_boundries.maxx", 10.0);
  double maxy = parent->declare_parameter("state_space_boundries.maxy", 10.0);
  double maxz = parent->declare_parameter("state_space_boundries.maxz", 10.0);
  double maxv = parent->declare_parameter("state_space_boundries.maxv", 1.5);
  se2_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
  z_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(1);
  auto v_bounds = std::make_shared<ompl::base::RealVectorBounds>(1);
  se2_bounds_->setLow(0, minx);
  se2_bounds_->setLow(1, miny);
  se2_bounds_->setHigh(0, maxx);
  se2_bounds_->setHigh(1, maxy);
  z_bounds_->setLow(0, minz);
  z_bounds_->setHigh(0, maxz);
  v_bounds->setLow(0, minv);
  v_bounds->setHigh(0, maxv);

  minv = parent->declare_parameter("control_boundries.minv", -0.5);
  double minw = parent->declare_parameter("control_boundries.minw", -0.5);
  maxv = parent->declare_parameter("control_boundries.maxv", 0.5);
  double maxw = parent->declare_parameter("control_boundries.maxw", 0.5);
  auto control_bounds = std::make_shared<ompl::base::RealVectorBounds>(2);
  control_bounds->setLow(0, minv);
  control_bounds->setLow(1, minw);
  control_bounds->setHigh(0, maxv);
  control_bounds->setHigh(1, maxw);

  if (selected_se2_space_name == "SE2") {
    se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::SE2;
  }
  else if (selected_se2_space_name == "DUBINS") {
    se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::DUBINS;
  }
  else {
    se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::REDDSSHEEP;
  }

  typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(new fcl::Box<float>(
    parent->get_parameter("robot_body_dimens.x").as_double(),
    parent->get_parameter("robot_body_dimens.y").as_double(),
    parent->get_parameter("robot_body_dimens.z").as_double()));

  fcl::CollisionObjectf robot_body_box_object(robot_body_box, fcl::Transform3f());
  robot_collision_object_ = std::make_shared<fcl::CollisionObjectf>(robot_body_box_object);
  get_map_client_node_ = std::make_shared<rclcpp::Node>("get_traversability_map_client_node");

  get_traversability_map_client_ =
    get_map_client_node_->create_client<vox_nav_msgs::srv::GetTraversabilityMap>(
      "get_traversability_map");

  is_map_ready_ = false;
  setupMap();

  state_space_ = std::make_shared<ompl::base::ElevationStateSpace>(
    se2_space_type_,
    rho_ /*only valid for dubins or reeds*/,
    false /*only valid for dubins*/);

  state_space_->as<ompl::base::ElevationStateSpace>()->setBounds(
    *se2_bounds_,
    *z_bounds_,
    *v_bounds);
  state_space_->setLongestValidSegmentFraction(0.1);

  control_state_space_ = std::make_shared<ompl::control::RealVectorControlSpace>(state_space_, 2);
  control_state_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(*control_bounds);

  control_simple_setup_ = std::make_shared<ompl::control::SimpleSetup>(control_state_space_);
  ompl::base::OptimizationObjectivePtr length_objective(new ompl::base::PathLengthOptimizationObjective(control_simple_setup_->getSpaceInformation()));
  control_simple_setup_->setOptimizationObjective(length_objective);
  control_simple_setup_->setStateValidityChecker(std::bind(&ElevationControlPlanner::isStateValid, this, std::placeholders::_1));
}

void ElevationControlPlanner::propagate(
  const ompl::control::SpaceInformation * si,
  const ompl::base::State * start,
  const ompl::control::Control * control,
  const double duration,
  ompl::base::State * result)
{
  const auto * ee_start = start->as<ompl::base::ElevationStateSpace::StateType>();
  const auto * ee_start_so2 = ee_start->as<ompl::base::SO2StateSpace::StateType>(0);
  const auto * ee_start_xyzv = ee_start->as<ompl::base::RealVectorStateSpace::StateType>(1);
  const double * ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

  auto x = ee_start_xyzv->values[0];
  auto y = ee_start_xyzv->values[1];
  auto z = ee_start_xyzv->values[2];
  auto v = ee_start_xyzv->values[3];
  auto yaw = ee_start_so2->value;

  result->as<ompl::base::ElevationStateSpace::StateType>()->setXYZV(
    x + duration * v * std::cos(yaw) /*X*/,
    y + duration * v * std::sin(yaw) /*Y*/,
    z /*Z*/,
    v + duration * ctrl[0] /*V*/);
  result->as<ompl::base::ElevationStateSpace::StateType>()->setSO2(
    yaw + duration *
            ctrl[1] /*W*/);

  si->enforceBounds(result);
}

std::vector<geometry_msgs::msg::PoseStamped> ElevationControlPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!is_map_ready_) {
    RCLCPP_WARN(
      logger_, "A valid Octomap has not been recived yet, Try later again.");
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }
  // set the start and goal states
  double start_yaw, goal_yaw, nan;
  vox_nav_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
  vox_nav_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);

  ompl::base::ScopedState<ompl::base::ElevationStateSpace>
    se3_start(state_space_),
    se3_goal(state_space_);

  geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_start;
  geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_goal;
  vox_nav_utilities::determineValidNearestStartGoal(nearest_elevated_surfel_to_start, nearest_elevated_surfel_to_goal, start, goal, elevated_surfel_cloud_);
  nearest_elevated_surfel_to_start.pose.orientation = start.pose.orientation;
  nearest_elevated_surfel_to_goal.pose.orientation = goal.pose.orientation;

  se3_start->setXYZV(
    nearest_elevated_surfel_to_start.pose.position.x,
    nearest_elevated_surfel_to_start.pose.position.y,
    nearest_elevated_surfel_to_start.pose.position.z, 0);
  se3_start->setSO2(start_yaw);

  se3_goal->setXYZV(
    nearest_elevated_surfel_to_goal.pose.position.x,
    nearest_elevated_surfel_to_goal.pose.position.y,
    nearest_elevated_surfel_to_goal.pose.position.z, 0);
  se3_goal->setSO2(goal_yaw);

  control_simple_setup_->setStartAndGoalStates(se3_start, se3_goal, 1.0);

  auto si = control_simple_setup_->getSpaceInformation();
  si->setMinMaxControlDuration(20, 30);
  si->setPropagationStepSize(0.025);

  control_simple_setup_->setStatePropagator(
    [this, si](const ompl::base::State * state, const ompl::control::Control * control,
      const double duration, ompl::base::State * result) {
      this->propagate(si.get(), state, control, duration, result);
    });

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  initializeSelectedControlPlanner(
    planner,
    planner_name_,
    si,
    logger_);

  control_simple_setup_->setPlanner(planner);
  control_simple_setup_->setup();
  control_simple_setup_->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = control_simple_setup_->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    ompl::control::PathControl solution_path(si);
    try {
      control_simple_setup_->getSolutionPath().printAsMatrix(std::cout);
      solution_path = control_simple_setup_->getSolutionPath();
    }
    catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
      RCLCPP_WARN(
        logger_, "Exception occured while retrivieng control solution path %s",
        e.what());
      control_simple_setup_->clear();
      return plan_poses;
    }

    RCLCPP_INFO(logger_, "A solution was found, the simplified solution path includes %ld poses.", solution_path.getStateCount());

    for (std::size_t path_idx = 0; path_idx < solution_path.getStateCount(); path_idx++) {
      const auto * cstate =
        solution_path.getState(path_idx)->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * cstate_so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * cstate_xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      double yaw = cstate_so2->value;
      double x = cstate_xyzv->values[0];
      double y = cstate_xyzv->values[1];
      double z = cstate_xyzv->values[2];
      tf2::Quaternion this_pose_quat;
      this_pose_quat.setRPY(0, 0, yaw);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = start.header.frame_id;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      pose.pose.orientation.x = this_pose_quat.getX();
      pose.pose.orientation.y = this_pose_quat.getY();
      pose.pose.orientation.z = this_pose_quat.getZ();
      pose.pose.orientation.w = this_pose_quat.getW();
      plan_poses.push_back(pose);
    }

    RCLCPP_INFO(
      logger_, "Found A plan with %ld poses", plan_poses.size());
  }
  else {
    RCLCPP_WARN(
      logger_, "No solution for requested path planning !");
  }

  control_simple_setup_->clear();
  return plan_poses;
}

bool ElevationControlPlanner::isStateValid(const ompl::base::State * state)
{
  const auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  // cast the abstract state type to the type we expect
  const auto * so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
  // extract the second component of the state and cast it to what we expect
  const auto * xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
  fcl::CollisionRequestf requestType(1, false, 1, false);
  // check validity of state Fdefined by pos & rot
  fcl::Vector3f translation(xyzv->values[0], xyzv->values[1], xyzv->values[2]);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, so2->value);
  fcl::Quaternionf rotation(myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());

  robot_collision_object_->setTransform(rotation, translation);

  fcl::CollisionResultf collisionWithFullMapResult;
  fcl::collide<float>(robot_collision_object_.get(), original_octomap_collision_object_.get(), requestType, collisionWithFullMapResult);

  return !collisionWithFullMapResult.isCollision();
}

void ElevationControlPlanner::setupMap()
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);

  while (!is_map_ready_ && rclcpp::ok()) {
    auto request = std::make_shared<vox_nav_msgs::srv::GetTraversabilityMap::Request>();

    while (!get_traversability_map_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          logger_,
          "Interrupted while waiting for the get_traversability_map service. Exiting");
        return;
      }
      RCLCPP_INFO(
        logger_,
        "get_traversability_map service not available, waiting and trying again");
    }

    auto result_future = get_traversability_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
          get_map_client_node_,
          result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "/get_traversability_map service call failed");
    }
    auto response = result_future.get();

    if (response->is_valid) {
      is_map_ready_ = true;
    }
    else {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        logger_, "Waiting for GetTraversabilityMap service to provide correct maps.");
      continue;
    }

    auto original_octomap_octree = dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(response->collision_octomap));
    std::shared_ptr<octomap::OcTree> original_octomap_octree_ = std::make_shared<octomap::OcTree>(*original_octomap_octree);
    delete original_octomap_octree;

    auto original_octomap_fcl_octree = std::make_shared<fcl::OcTreef>(original_octomap_octree_);
    original_octomap_collision_object_ = std::make_shared<fcl::CollisionObjectf>(std::shared_ptr<fcl::CollisionGeometryf>(original_octomap_fcl_octree));

    pcl::fromROSMsg(response->traversable_elevated_cloud, *elevated_surfel_cloud_);

    RCLCPP_INFO(
      logger_,
      "Recieved a valid Octomap with %ld nodes, A FCL collision tree will be created from this "
      "octomap for state validity (aka collision check)",
      original_octomap_octree_->size());
  }
}
}  // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::ElevationControlPlanner, vox_nav_planning::PlannerCore)
