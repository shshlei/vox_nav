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


#include "vox_nav_planning/box2d_planner_core.hpp"
#include <vox_nav_utilities/planner_helpers.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

// OMPL Base
#include <ompl/base/Planner.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>

// OMPLAPP
#include <omplapp/geometry/detail/Box2dStateValidityChecker.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

#include <memory>
#include <string>
#include <vector>

namespace rvt = rviz_visual_tools;

namespace vox_nav_planning
{
void Box2dPlannerCore::initialize(rclcpp::Node * parent)
{
  pub_nodes_ = parent->declare_parameter("pub_nodes", false);
  if (pub_nodes_) {
    visual_tools_.reset(new rvt::RvizVisualizationTools("map", "planning/nodes", parent));

    // create publisher before waiting
    visual_tools_->loadMarkerPub();
    bool has_sub = visual_tools_->waitForMarkerSub(10.0);
    if (!has_sub) {
      RCLCPP_INFO(logger_, "/rviz_visual_tools does not have a subscriber after 10s. Visualizations may be lost!");
    }
    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
  }

  // Declare this node's parameters
  std::string planner_name = parent->declare_parameter("planner_name", "RRTStar");
  planner_timeout_ = parent->declare_parameter("planner_timeout", 5.0);
  interpolation_parameter_ = parent->declare_parameter("interpolation_parameter", 50);
  do_simplification_ = parent->declare_parameter("do_simplification", true);
  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name.c_str());

  double minx = parent->declare_parameter("state_space_boundries.minx", -50.0);
  double miny = parent->declare_parameter("state_space_boundries.miny", -10.0);
  double maxx = parent->declare_parameter("state_space_boundries.maxx", 50.0);
  double maxy = parent->declare_parameter("state_space_boundries.maxy", 10.0);

  std::shared_ptr<ompl::base::RealVectorBounds> se2_bounds;
  se2_bounds = std::make_shared<ompl::base::RealVectorBounds>(2);
  se2_bounds->setLow(0, minx);
  se2_bounds->setLow(1, miny);
  se2_bounds->setHigh(0, maxx);
  se2_bounds->setHigh(1, maxy);

  std::string selected_se2_space_name = parent->declare_parameter("se2_space", "REEDS");
  double rho = parent->declare_parameter("rho", 1.5);
  if (selected_se2_space_name == "SE2") {
    state_space_ = std::make_shared<ompl::base::SE2StateSpace>();
  }
  else if (selected_se2_space_name == "DUBINS") {
    state_space_ = std::make_shared<ompl::base::DubinsStateSpace>(rho, false);
  }
  else {
    state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(rho);
  }

  state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*se2_bounds);
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);
  // objective is to minimize the planned path
  auto si = simple_setup_->getSpaceInformation();
  ompl::base::OptimizationObjectivePtr objective(new ompl::base::PathLengthOptimizationObjective(si));
  simple_setup_->setOptimizationObjective(objective);

  std::string env_shapes_file = parent->declare_parameter("env_shapes_file", "");
  std::string robot_shape_file = parent->declare_parameter("robot_shape_file", "");

  std::shared_ptr<ompl::app::Box2dStateValidityChecker> svc = std::make_shared<ompl::app::Box2dStateValidityChecker>(si, state_space_);
  svc->setEnvironmentFile(env_shapes_file);
  svc->addRobotShape(robot_shape_file);

  si->setStateValidityChecker(svc);
  double check_resolution = parent->declare_parameter("check_resolution", 0.01);
  si->setStateValidityCheckingResolution(check_resolution);
  si->setup();

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  vox_nav_utilities::initializeSelectedPlanner(planner, planner_name, si, logger_);
  simple_setup_->setPlanner(planner);

  // TODO
  pose_x_ = parent->declare_parameter("pose_x", std::vector<double>{});
  pose_y_ = parent->declare_parameter("pose_y", std::vector<double>{});
  pose_yaw_ = parent->declare_parameter("pose_yaw", std::vector<double>{});
}

nav_2d_msgs::msg::Path2D Box2dPlannerCore::createPlan(const nav_2d_msgs::msg::Pose2DStamped & start, const nav_2d_msgs::msg::Pose2DStamped & goal)
{
  ompl::base::ScopedState<ompl::base::SE2StateSpace> se2_start(state_space_), se2_goal(state_space_);
  se2_start->setX(start.pose.x);
  se2_start->setY(start.pose.y);
  se2_start->setYaw(start.pose.theta);
  se2_goal->setX(goal.pose.x);
  se2_goal->setY(goal.pose.y);
  se2_goal->setYaw(goal.pose.theta);

  simple_setup_->clear();
  simple_setup_->setStartAndGoalStates(se2_start, se2_goal);
  simple_setup_->setup();
  // print the settings for this space
  // simple_setup_->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = simple_setup_->solve(planner_timeout_);
  nav_2d_msgs::msg::Path2D plan_poses;
  plan_poses.header.frame_id = start.header.frame_id;
  plan_poses.header.stamp = rclcpp::Clock().now();

  if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION) {
    /*
    if (do_simplification_) simple_setup_->simplifySolution();
    ompl::geometric::PathGeometric solution_path = simple_setup_->getSolutionPath();
    solution_path.interpolate(interpolation_parameter_);
    for (std::size_t path_idx = 0; path_idx < solution_path.getStateCount(); path_idx++) {
      // cast the abstract state type to the type we expect
      const ompl::base::SE2StateSpace::StateType * se2_state =
        solution_path.getState(path_idx)->as<ompl::base::SE2StateSpace::StateType>();
      geometry_msgs::msg::Pose2D pose;
      pose.x = se2_state->getX();
      pose.y = se2_state->getY();
      pose.theta = se2_state->getYaw();
      plan_poses.poses.push_back(pose);
    }
    */
    for (std::size_t path_idx = 0; path_idx < pose_x_.size(); path_idx++) { // TODO
      geometry_msgs::msg::Pose2D pose;
      pose.x = pose_x_[path_idx];
      pose.y = pose_y_[path_idx];
      pose.theta = pose_yaw_[path_idx];
      plan_poses.poses.push_back(pose);
    }
    RCLCPP_INFO(logger_, "Found A plan with %ld poses", plan_poses.poses.size());
  }
  else {
    RCLCPP_WARN(logger_, "No solution for requested path planning !");
  }

  if (pub_nodes_) {
    visual_tools_->deleteAllMarkers();
    std::size_t id = 0, lid = 0;

    std_msgs::msg::ColorRGBA color;
    // color.r = 0.717647059; // red
    // color.g = 0.247058824;
    // color.b = 0.266666667;
    // color.a = 1.0;
    color.r = 0.098039216; // green
    color.g = 0.701960784;
    color.b = 0.250980392;
    color.a = 1.0;

    ompl::base::PlannerData pd(simple_setup_->getSpaceInformation());
    simple_setup_->getPlannerData(pd);
    for (unsigned int i = 0; i < pd.numVertices(); i++) {
      std::vector<unsigned int> edgeList;
      if (!pd.getEdges(i, edgeList)) continue;

      const ompl::base::State * statei = pd.getVertex(i).getState();
      const ompl::base::SE2StateSpace::StateType * se2_statei = statei->as<ompl::base::SE2StateSpace::StateType>();

      if (se2_statei->getX() < 0.0 && se2_statei->getY() < -5.0) continue;

      if (se2_statei->getX() < 13.0 && se2_statei->getY() < -7.0) continue;
      if (se2_statei->getX() < 10.0 && se2_statei->getY() < -5.5) continue;

      if (se2_statei->getX() > 6.0 && se2_statei->getY() > 1.5) continue;
      if (se2_statei->getX() > 12.0 && se2_statei->getY() > -1.2) continue;

      // if (se2_statei->getX() > 8.0 && se2_statei->getY() > -0.5) continue;

      if (se2_statei->getX() > 18.0 && se2_statei->getY() > -2.5) continue;

      geometry_msgs::msg::Point pointi;
      pointi.x = se2_statei->getX();
      pointi.y = se2_statei->getY();
      pointi.z = -0.4;
      visual_tools_->publishSphere(pointi, rvt::MAGENTA, 0.1, "Sphere", id++);

      for (unsigned int j : edgeList) {
        const ompl::base::State * statej = pd.getVertex(j).getState();
        const ompl::base::SE2StateSpace::StateType * se2_statej = statej->as<ompl::base::SE2StateSpace::StateType>();

        if (se2_statej->getX() < 0.0 && se2_statej->getY() < -5.0) continue;

        if (se2_statej->getX() < 13.0 && se2_statej->getY() < -7.0) continue;
        if (se2_statej->getX() < 10.0 && se2_statej->getY() < -5.5) continue;

        if (se2_statej->getX() > 6.0 && se2_statej->getY() > 1.5) continue;
        if (se2_statej->getX() > 12.0 && se2_statej->getY() > -1.2) continue;

        // if (se2_statej->getX() > 8.0 && se2_statej->getY() > -0.5) continue;

        if (se2_statej->getX() > 18.0 && se2_statej->getY() > -2.5) continue;

        geometry_msgs::msg::Point pointj;
        pointj.x = se2_statej->getX();
        pointj.y = se2_statej->getY();
        pointj.z = -0.4;
        visual_tools_->publishSphere(pointj, rvt::MAGENTA, 0.1, "Sphere", id++);
        visual_tools_->publishLine(pointi, pointj, color, rvt::LARGE, lid++);
      }
    }

    std::vector<geometry_msgs::msg::Point> path;
    /*
    for (const auto pose : plan_poses.poses) { // TODO
      geometry_msgs::msg::Point pointj;
      pointj.x = pose.x;
      pointj.y = pose.y;
      pointj.z = -0.4;
      visual_tools_->publishSphere(pointj, rvt::MAGENTA, 0.1, "Sphere", id++);
      path.push_back(pointj);
    }
    */
    for (std::size_t path_idx = 0; path_idx < pose_x_.size(); path_idx++) { // TODO
      geometry_msgs::msg::Point pointj;
      pointj.x = pose_x_[path_idx];
      pointj.y = pose_y_[path_idx];
      pointj.z = -0.4;
      visual_tools_->publishSphere(pointj, rvt::MAGENTA, 0.1, "Sphere", id++);
      path.push_back(pointj);
    }
    visual_tools_->publishPath(path, rvt::CYAN, 0.2);
    visual_tools_->trigger();
  }

  return plan_poses;
}
}  // namespace vox_nav_planning
