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

#include "vox_nav_planning/box2d_trajectory_server.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <box2d_collision/b2_distance.h>

#include <psopt/interpolation.hpp>
#include <psopt/solver.hpp>

#include <memory>
#include <string>
#include <vector>

namespace rvt = rviz_visual_tools;

class APCallback : public b2NaiveCallback
{
public:
  APCallback(double activeDist, const b2RectangleShape * rect, const b2OBB * obb) : b2NaiveCallback()
  {
    activeDist_ = minDist_ = activeDist;

    rect_ = rect;

    xf_.setIdentity();
    xf_.linear() = obb->axis();
    xf_.translation() = obb->center();

    invxf_.setIdentity();
    invxf_.linear() = obb->axis().transpose();
    invxf_.translation() = -invxf_.linear() * obb->center();
  }

  ~APCallback() override = default;

  bool ReportCollision(b2Fixture* fixture) override
  {
    if (fixture->GetBody()->IsActive()) return false;
    b2Scalar d;
    b2Vec2 p1, p2;
    dist_.SignedDistance(rect_, fixture->GetShape(), invxf_ * fixture->GetGlobalTransform(), &d, &p1, &p2);
    if (d >= activeDist_) return false;
    if (d < minDist_) minDist_ = d;

    b2Vec2 normal = p2 - p1;
    if (d < 0.0) normal = -normal;
    if (abs(d) < 1.e-6) normal = p1;

    b2Vec2 tau = b2Cross(1.0, normal).normalized();
    psopt::ActivePoint2D<double> apoint;
    apoint.normal = 100.0 * normal.normalized(); // TODO
    apoint.b = xf_ * p2;
    b2Vec2 a1 = p1 - tau + apoint.normal;
    b2Vec2 a2 = p1 + tau + apoint.normal;
    apoint.a = a2 - a1;
    apoint.det = a1.x() * a2.y() - a1.y() * a2.x();
    activePoints_.push_back(apoint);

    return false; // Never stop early
  }

  double getMinDist() const
  {
    return minDist_;
  }

  std::vector<psopt::ActivePoint2D<double>> & getActivePoints()
  {
    return activePoints_;
  }

private:

  double activeDist_, minDist_;

  // Distance Calculation
  b2ShapeDistance dist_;

  // Local vehicle rectangle
  const b2RectangleShape* rect_;

  // Vehicle Global Transform
  b2Transform xf_, invxf_;

  std::vector<psopt::ActivePoint2D<double>> activePoints_;
};

namespace vox_nav_planning
{
Box2dTrajectoryServer::Box2dTrajectoryServer()
: Node("vox_nav_planning_box2d_trajectory_server")
{
  this->action_server_ = rclcpp_action::create_server<Compute2DStartToGoalTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "box2d_compute_start_to_goal_trajectory",
    std::bind(&Box2dTrajectoryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Box2dTrajectoryServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&Box2dTrajectoryServer::handle_accepted, this, std::placeholders::_1));

  // create publisher before waiting
  visual_tools_.reset(new rvt::RvizVisualizationTools("map", "planning/trajectory", this));
  visual_tools_->loadMarkerPub();
  bool has_sub = visual_tools_->waitForMarkerSub(10.0);
  if (!has_sub) {
    RCLCPP_INFO(get_logger(), "/rviz_visual_tools does not have a subscriber after 10s. Visualizations may be lost!");
  }
  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // initialize problem
  initialize_problem();
}

Box2dTrajectoryServer::~Box2dTrajectoryServer()
{
  action_server_.reset();
  delete rect_;
  delete info_;
  delete problem_;
  rect_ = nullptr;
  info_ = nullptr;
  problem_ = nullptr;
}

// Start To Goal
rclcpp_action::GoalResponse Box2dTrajectoryServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Compute2DStartToGoalTrajectory::Goal> /*goal*/)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request in order to compute a start to goal trajectory");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Box2dTrajectoryServer::handle_cancel(const std::shared_ptr<GoalHandleCompute2DStartToGoalTrajectory> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Box2dTrajectoryServer::handle_accepted(const std::shared_ptr<GoalHandleCompute2DStartToGoalTrajectory> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Box2dTrajectoryServer::computePlan, this, std::placeholders::_1), goal_handle}.detach();
}

void Box2dTrajectoryServer::computePlan(const std::shared_ptr<GoalHandleCompute2DStartToGoalTrajectory> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Compute2DStartToGoalTrajectory::Feedback>();
  auto result = std::make_shared<Compute2DStartToGoalTrajectory::Result>();

  auto start_time = steady_clock_.now();
  result->trajectory = getPlan(goal->path);
  auto elapsed_time = steady_clock_.now() - start_time;
  if (result->trajectory.poses.size() == 0) {
    RCLCPP_WARN(get_logger(), "Planning algorithm failed to generate a valid trajectory");
    return;
  }

  // Check if there is a cancel request
  if (goal_handle->is_canceling()) {
    result->trajectory = nav_2d_msgs::msg::Path2D();
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
  }
}

void Box2dTrajectoryServer::initialize_problem()
{
  num_iterative_ = declare_parameter("num_iterative", 1);
  RCLCPP_INFO(get_logger(), "Num iterative %d", num_iterative_);

  // vehicle
  active_dist_ = declare_parameter<double>("active_dist", 2.0);
  RCLCPP_INFO(get_logger(), "active_dist %.4f", active_dist_);
  vehicle_half_length_ = declare_parameter("vehicle_half_length", 0.3);
  vehicle_half_width_ = declare_parameter("vehicle_half_width", 0.3);
  // local vehicle rectangle
  rect_ = new b2RectangleShape(vehicle_half_length_, vehicle_half_width_);

  // collision environment
  std::string env_shapes_file = declare_parameter("env_shapes_file", "");
  RCLCPP_INFO(get_logger(), "env_shapes_file %s", env_shapes_file.c_str());
  svc_ = std::make_shared<b2BVHManager>();
  svc_->setEnvironmentFile(env_shapes_file);

  // basic problem definition
  psopt::MultiSegmentData msdata;
  msdata.nsegments = 1;
  msdata.nstates = 3;
  msdata.ncontrols = 2;
  msdata.nparameters = 0;
  msdata.ninitial_events = 4;
  msdata.nfinal_events = 4;
  msdata.npaths = 0;
  msdata.continuous_controls = true;
  msdata.paths_along_trajectory = false;
  msdata.parameters_along_trajectory = false;
  msdata.treat_dynamics_as_cost = false;
  msdata.nnodes.push_back(20);
  info_ = new psopt::ProblemInfo<double>(msdata);
  info_->setLinearSolver("ma57");
  info_->setTolerance(1.e-8);

  // optimization problem
  problem_ = new DifferentialDriving<double>(info_);

  // bounds
  double minx = declare_parameter("state_space_boundries.minx", -50.0);
  double miny = declare_parameter("state_space_boundries.miny", -10.0);
  double maxx = declare_parameter("state_space_boundries.maxx", 50.0);
  double maxy = declare_parameter("state_space_boundries.maxy", 10.0);
  double maxv = declare_parameter("control_space_boundries.maxv", 3.0);
  double maxo = declare_parameter("control_space_boundries.maxomega", 1.0);

  std::vector<double> slower{minx, miny, -1.1 * M_PI};
  std::vector<double> supper{maxx, maxy, 1.1 * M_PI};
  std::vector<double> clower{-maxv, -maxo};
  std::vector<double> cupper{maxv, maxo};
  for (std::size_t i = 0; i < info_->getPhaseNumbers(); i++)
  {
    info_->setPhaseLowerBoundStates(slower, i);
    info_->setPhaseUpperBoundStates(supper, i);
    info_->setPhaseLowerBoundControls(clower, i);
    info_->setPhaseUpperBoundControls(cupper, i);
  }

  info_->setPhaseLowerBoundStartTime(0.0);
  info_->setPhaseUpperBoundStartTime(0.0);
  info_->setPhaseLowerBoundEndTime(2.0);
  info_->setPhaseUpperBoundEndTime(200.0);
  for (std::size_t i = 1; i < info_->getPhaseNumbers(); i++)
  {
    info_->setPhaseLowerBoundStartTime(2.0, i);
    info_->setPhaseUpperBoundStartTime(200.0, i);
    info_->setPhaseLowerBoundEndTime(2.0, i);
    info_->setPhaseUpperBoundEndTime(200.0, i);
  }
}

nav_2d_msgs::msg::Path2D Box2dTrajectoryServer::getPlan(const nav_2d_msgs::msg::Path2D & path)
{
  std::size_t expected_nnodes = path.poses.size();
  std::vector<double> trajx;
  std::vector<double> trajy;
  std::vector<double> trajt;
  trajx.reserve(expected_nnodes);
  trajy.reserve(expected_nnodes);
  trajt.reserve(expected_nnodes);
  for (std::size_t i = 0; i < expected_nnodes; i++)
  {
    trajx.push_back(path.poses[i].x);
    trajy.push_back(path.poses[i].y);
    trajt.push_back(path.poses[i].theta);
  }

  // interpolated states
  std::vector<double> interp_trajx;
  std::vector<double> interp_trajy;
  std::vector<double> interp_trajt;
  if (expected_nnodes < 20)
  {
    Eigen::VectorXd etime1 = Eigen::VectorXd::LinSpaced(expected_nnodes, 0.0, 1.0);
    std::vector<double> vtime1(etime1.data(), etime1.data() + expected_nnodes);
    expected_nnodes = 20;
    Eigen::VectorXd etime2 = Eigen::VectorXd::LinSpaced(expected_nnodes, 0.0, 1.0);
    std::vector<double> vtime2(etime2.data(), etime2.data() + expected_nnodes);
    psopt::LinearInterpolation<double, double> linearInterp(vtime1, vtime2);
    interp_trajx = linearInterp.interpolate(trajx);
    interp_trajy = linearInterp.interpolate(trajy);
    interp_trajt = linearInterp.interpolate(trajt);
  }
  else
  {
    // interpolated states
    interp_trajx.swap(trajx);
    interp_trajy.swap(trajy);
    interp_trajt.swap(trajt);
  }
  info_->setPhaseNumberNodes(expected_nnodes);

  // initialization other trajectory parameters
  std::vector<double> interp_trajv(expected_nnodes, 0.0);
  std::vector<double> interp_trajomega(expected_nnodes, 0.0);
  for (std::size_t i = 1; i < expected_nnodes; i++)
  {
    interp_trajv[i] = 0.1 * std::hypot(interp_trajx[i] - interp_trajx[i - 1], interp_trajy[i] - interp_trajy[i - 1]); 
    if (interp_trajx[i] < interp_trajx[i - 1]) interp_trajv[i] = -interp_trajv[i];
    interp_trajomega[i] = 0.1 * (interp_trajt[i] - interp_trajt[i - 1]); 
  }

  // start end event bounds
  double x0 = interp_trajx[0];
  double y0 = interp_trajy[0];
  double theta0 = interp_trajt[0];
  double xf = interp_trajx.back();
  double yf = interp_trajy.back();
  double thetaf = interp_trajt.back();
  if (info_->getPhaseNumbers() == 1)
  {
    double epsf = 0.1;
    double epsthetaf = 0.035;
    std::vector<double> elower{x0, y0, theta0, 0.0, xf - epsf, yf - epsf, thetaf - epsthetaf, -epsf};
    std::vector<double> eupper{x0, y0, theta0, 0.0, xf + epsf, yf + epsf, thetaf + epsthetaf, epsf};
    info_->setPhaseLowerBoundEvents(elower);
    info_->setPhaseUpperBoundEvents(eupper);
  }
  else
  {
    std::vector<double> elower{x0, y0, theta0, 0.0};
    std::vector<double> eupper{x0, y0, theta0, 0.0};
    info_->setPhaseLowerBoundEvents(elower);
    info_->setPhaseUpperBoundEvents(eupper);

    double epsf = 0.001;
    double epsthetaf = 0.035;
    std::vector<double> felower{xf - epsf, yf - epsf, thetaf - epsthetaf, -epsf};
    std::vector<double> feupper{xf + epsf, yf + epsf, thetaf + epsthetaf, epsf};
    info_->setPhaseLowerBoundEvents(felower, info_->getPhaseNumbers() - 1);
    info_->setPhaseUpperBoundEvents(feupper, info_->getPhaseNumbers() - 1);
  }

  const std::size_t nphases = info_->getPhaseNumbers();
  const std::size_t nstates = info_->getPhaseNumberStates();
  const std::size_t ncontrols = info_->getPhaseNumberControls();

  // initial guess states
  psopt::Solver<double> solver;
  solver.setPhaseNumbers(nphases);
  std::size_t start = 0;
  for (std::size_t k = 0; k < nphases; k++)
  {
    std::vector<double> guess_states, guess_controls;
    guess_states.reserve(nstates * info_->getPhaseNumberNodes(k));
    guess_controls.reserve(ncontrols * info_->getPhaseNumberNodes(k));
    for (std::size_t i = 0; i < info_->getPhaseNumberNodes(k); i++)
    {
      std::size_t offset = start + i;
      guess_states.push_back(interp_trajx[offset]);
      guess_states.push_back(interp_trajy[offset]);
      guess_states.push_back(interp_trajt[offset]);
      guess_controls.push_back(interp_trajv[offset]);
      guess_controls.push_back(interp_trajomega[offset]);
      for (std::size_t j = 3; j < nstates; j++)
        guess_states.push_back(0.0);
      for (std::size_t j = 2; j < ncontrols; j++)
        guess_controls.push_back(0.0);
    }
    start += (info_->getPhaseNumberNodes(k) - 1);
    info_->setPhaseGuessStates(guess_states, k);
    info_->setPhaseGuessControls(guess_controls, k);
    solver.setPhaseStates(guess_states, k);
  }
  interp_trajx.clear();
  interp_trajy.clear();
  interp_trajt.clear();
  interp_trajv.clear();
  interp_trajomega.clear();

  // guess time
  double stime = 0.0;
  double dtime = 10.0 / nphases;
  for (std::size_t i = 0; i < nphases; i++)
  {
    info_->setPhaseGuessTime(stime, stime + dtime, i);
    stime += dtime;
  }

  // obb extent
  b2Vec2 extent(vehicle_half_length_ + active_dist_, vehicle_half_width_ + active_dist_);
  b2Vec2 iextent(vehicle_half_length_, vehicle_half_width_);

  int sol = 0;
  bool success = true;
  bool use_linearized_system = false;
  double dynamics_cost_weight = 10.0;
  double best_cost = std::numeric_limits<double>::max();
  while (success && sol < num_iterative_)
  {
    sol++;
    // box and ap constraints
    std::vector<std::vector<psopt::ActivePoints2D<double>>> phaseActivePoints; 
    phaseActivePoints.reserve(nphases);
    for (std::size_t k = 0; k < nphases; k++)
    {
      // get active points
      std::size_t nactive = 0, nactive_state = 0;
      std::vector<double> lower_box_bounds, upper_box_bounds;
      lower_box_bounds.reserve(2 * info_->getPhaseNumberNodes(k));
      upper_box_bounds.reserve(2 * info_->getPhaseNumberNodes(k));
      std::vector<psopt::ActivePoints2D<double>> activePoints; 
      activePoints.reserve(info_->getPhaseNumberNodes(k));
      const double * states = solver.getPhaseStates(k).data() + nstates;
      for (std::size_t i = 1; i < info_->getPhaseNumberNodes(k); i++)
      {
        double x = states[0];
        double y = states[1];
        double yaw = states[2];
        states += nstates;

        b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
        b2Vec2 center(x, y);
        b2OBB obb(axis, center, extent);
        APCallback callback(active_dist_, rect_, &obb);
        svc_->Collide(&callback, obb);
        double bound = active_dist_;
        if (callback.getMinDist() < active_dist_) bound = std::max(callback.getMinDist(), 0.5 * active_dist_);
        lower_box_bounds.push_back(x - bound);
        lower_box_bounds.push_back(y - bound);
        upper_box_bounds.push_back(x + bound);
        upper_box_bounds.push_back(y + bound);

        if (callback.getActivePoints().empty()) continue;
        psopt::ActivePoints2D<double> apoints; 
        apoints.index = i;
        apoints.activePoints.swap(callback.getActivePoints());
        activePoints.push_back(apoints);
        nactive_state++;
        nactive += apoints.activePoints.size();
      }

      info_->setPhaseNumberPaths(lower_box_bounds.size() + nactive, k);
      for (std::size_t i = 0; i < nactive; i++)
      {
        lower_box_bounds.push_back(1.0005); 
        upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
      }
      info_->setPhaseLowerBoundPaths(lower_box_bounds, k);
      info_->setPhaseUpperBoundPaths(upper_box_bounds, k);
      phaseActivePoints.push_back(activePoints);
    }

    // solve with current box and ap constraints
    problem_->setActivePoints(phaseActivePoints);
    if (info_->getTreatDynamicsAsCost()) info_->setDynamicsCostWeight(dynamics_cost_weight);
    if (use_linearized_system)
    {
      info_->setUseLinearizedDae(true);
      problem_->setUpLinearizedParameters(info_->getVariables().data());
      success = solver.solve(problem_);
      if (!success)
      {
        RCLCPP_ERROR(get_logger(), "Failed to find a feasible trajectory!");
        break;
      }
      info_->setUseLinearizedDae(false);
    }
    success = solver.solve(problem_);
    if (!success)
    {
      RCLCPP_ERROR(get_logger(), "Failed to find a feasible trajectory!");
      break;
    }

    // repair current local optimal trajectory
    while (success)
    {
      bool error = false;
      for (std::size_t k = 0; k < nphases; k++)
      {
        std::size_t nnactive = 0, nnactive_state = 0;
        std::vector<psopt::ActivePoints2D<double>> tempPoints;
        tempPoints.reserve(info_->getPhaseNumberNodes(k));
        const double * states = solver.getPhaseStates(k).data() + nstates;
        for (std::size_t i = 1; i < info_->getPhaseNumberNodes(k); i++)
        {
          double x = states[0];
          double y = states[1];
          double yaw = states[2];
          states += nstates;

          b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
          b2Vec2 center(x, y);
          b2OBB obb(axis, center, iextent);
          APCallback callback(0.0001, rect_, &obb);
          svc_->Collide(&callback, obb);

          if (callback.getActivePoints().empty()) continue;
          psopt::ActivePoints2D<double> apoints; 
          apoints.index = i;
          apoints.activePoints.swap(callback.getActivePoints());
          tempPoints.push_back(apoints);
          nnactive_state += 1;
          nnactive += apoints.activePoints.size();
        }
        if (nnactive_state == 0) continue;
        error = true;
        std::vector<double> lower_box_bounds = info_->getPhaseLowerBoundPaths(k);
        std::vector<double> upper_box_bounds = info_->getPhaseUpperBoundPaths(k);
        info_->setPhaseNumberPaths(lower_box_bounds.size() + nnactive, k);
        for (std::size_t i = 0; i < nnactive; i++)
        {
          lower_box_bounds.push_back(1.0005); 
          upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
        }
        info_->setPhaseLowerBoundPaths(lower_box_bounds, k);
        info_->setPhaseUpperBoundPaths(upper_box_bounds, k);
        phaseActivePoints[k].insert(phaseActivePoints[k].end(), tempPoints.begin(), tempPoints.end());
      }
      if (!error) break;
      RCLCPP_WARN(get_logger(), "Current local optimal trajectory is infeasible, try to fix it!");
      /*
         if (use_linearized_system)
         {
         info_->setUseLinearizedDae(true);
         success = solver.solve(problem_);
         if (!success) break;
         info_->setUseLinearizedDae(false);
         }
         */
      problem_->setActivePoints(phaseActivePoints);
      success = solver.solve(problem_);
    }
    if (!success)
    {
      RCLCPP_ERROR(get_logger(), "Failed to find a feasible trajectory!");
      break;
    }
    if (info_->getTreatDynamicsAsCost())
    {
      RCLCPP_INFO(get_logger(), "Current cost %0.4f, current infeasiblity %0.4f, current w_penalty %0.4f", solver.getCost(), info_->getDynamicsCost(), dynamics_cost_weight);
      if (info_->getDynamicsCost() < 1.e-2) // TODO
      {
        RCLCPP_INFO(get_logger(), "Found the best trajectory!");
        break;
      }
      dynamics_cost_weight *= 10.0;
    }
    else
      RCLCPP_INFO(get_logger(), "Current cost %0.4f", solver.getCost());
    if (std::abs(best_cost - solver.getCost()) < 1.e-2 * std::abs(best_cost)) // TODO
    {
      RCLCPP_INFO(get_logger(), "Found the best trajectory!");
      break;
    }
    if (solver.getCost() <= best_cost)
    {
      best_cost = solver.getCost();
    }
    // use_linearized_system = true; // TODO
    // info_->setUseLinearizedDae(true);
    // problem_->setUpLinearizedParameters(info_->getVariables().data());
  }

  nav_2d_msgs::msg::Path2D trajectory;
  trajectory.header.frame_id = path.header.frame_id;
  trajectory.header.stamp = rclcpp::Clock().now();
  if (!success) return trajectory;

  std::vector<geometry_msgs::msg::Point> sol_path;
  for (std::size_t k = 0; k < nphases; k++)
  {
    const double * states = solver.getPhaseStates(k).data();
    for (std::size_t j = 0; j < info_->getPhaseNumberNodes(k); j++)
    {
      geometry_msgs::msg::Pose2D pose;
      pose.x = states[0];
      pose.y = states[1];
      pose.theta = states[2];
      trajectory.poses.push_back(pose);
      states += nstates;

      geometry_msgs::msg::Point point;
      point.x = pose.x;
      point.y = pose.y;
      point.z = -0.4;
      sol_path.push_back(point);
    }
  }

  visual_tools_->deleteAllMarkers();
  visual_tools_->publishPath(sol_path, rvt::YELLOW, 0.2);
  visual_tools_->trigger();

  /*
  if (success)
  {
    std::ofstream ofs;
    ofs.open(boost::str(boost::format("states_%i.txt") % sol).c_str());
    for (std::size_t k = 0; k < nphases; k++)
    {
      const double * states = solver.getPhaseStates(k).data();
      for (std::size_t j = 0; j < info_->getPhaseNumberNodes(k); j++)
      {
        for (std::size_t i = 0; i < nstates; i++)
        {
          ofs << states[i] << " ";
        }
        states += nstates;
        ofs << std::endl;
      }
    }
    ofs.close();

    ofs.open("controls.txt");
    for (std::size_t i = 0; i < ncontrols; i++)
    {
      for (std::size_t j = 0; j < nphases; j++)
      {
        std::vector<double> traj = solver.getPhaseTrajectoryControls(ncontrols, info_->getPhaseNumberNodes(j), i, j);
        if (j > 0) traj.erase(traj.begin());
        for (double data : traj) ofs << data << " ";
      }
      ofs << std::endl;
    }
    ofs.close();

    ofs.open("time.txt");
    for (std::size_t j = 0; j < nphases; j++)
    {
      std::vector<double> traj = solver.getPhaseTime(j);
      if (j > 0) traj.erase(traj.begin());
      for (double data : traj) ofs << data << " ";
    }
    ofs.close();
  }
  */

  return trajectory;
}
}  // namespace vox_nav_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_planning::Box2dTrajectoryServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
