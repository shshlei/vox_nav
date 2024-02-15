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

#include "vox_nav_planning/native_planners/LQRPlanner.hpp"

#include <vox_nav_utilities/elevation_state_space.hpp>

#include <ompl/control/PathControl.h>

ompl::control::LQRPlanner::LQRPlanner(const SpaceInformationPtr & si)
: ompl::base::Planner(si, "LQRPlanner")
{
  specs_.approximateSolutions = true;
}

ompl::control::LQRPlanner::~LQRPlanner()
{
  freeMemory();
}

void ompl::control::LQRPlanner::setup()
{
  ompl::base::Planner::setup();

  // ros2 node to publish rrt nodes
  // node_ = std::make_shared<rclcpp::Node>("LQRPlanner_rclcpp_node");
  // rrt_nodes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/rrtstar/nodes", rclcpp::SystemDefaultsQoS());
}

void ompl::control::LQRPlanner::clear()
{
  Planner::clear();
  freeMemory();
}

void ompl::control::LQRPlanner::freeMemory()
{
}

ompl::base::PlannerStatus ompl::control::LQRPlanner::solve(
  const ompl::base::PlannerTerminationCondition & ptc)
{
  checkValidity();
  // base::Goal * goal = pdef_->getGoal().get();
  // auto * goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

  // get the goal node and state
  auto * goal_state = si_->allocState();
  while (const ompl::base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_state, goal);
  }

  // get start node and state,push  the node inton nn_ as well
  auto * start_state = si_->allocState();
  while (const ompl::base::State * st = pis_.nextStart()) {
    si_->copyState(start_state, st);
  }

  if (!start_state) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ompl::base::PlannerStatus::INVALID_START;
  }

  OMPL_INFORM("%s: Starting planning with", getName().c_str());

  std::vector<ompl::base::State *> resulting_path;

  while (ptc == false) {
    // try to compute plan as long as we are allowed
    compute_LQR_plan(start_state, goal_state, resulting_path);
    if (resulting_path.size() > 2) {
      break; /*Found a solution with at least some pose*/
    }
  }

  bool solved = false;
  bool approximate = false;

  if (resulting_path.size() > 2) {
    solved = true;

    /* set the solution path */
    auto path(std::make_shared<PathControl>(si_));
    for (auto i : resulting_path) {
      if (i) {
        path->append(i);
      }
    }
    solved = true;
    pdef_->addSolutionPath(path, approximate, 0.0 /*approxdif*/, getName());
    OMPL_INFORM("Found solution with %u poses", resulting_path.size());
  }
  else {
    OMPL_WARN("%s: Failed to cretae a plan after", getName().c_str());
  }
  clear();
  return {solved, approximate};
}

void ompl::control::LQRPlanner::getPlannerData(ompl::base::PlannerData & data) const
{
  Planner::getPlannerData(data);
}

std::tuple<Eigen::Matrix2d, Eigen::Vector2d, Eigen::Matrix2d, double> ompl::control::LQRPlanner::getABQR()
{
  Eigen::Matrix2d A(2, 2);
  A << 0, -v_r_,
    0, 0;
  Eigen::Vector2d B(0, -v_r_ / L_);
  Eigen::Matrix2d Q(2, 2);
  Q << q1_, 0,
    0, q2_;
  double R(r_);
  return std::make_tuple(A, B, Q, R);
}

std::vector<ompl::base::State *> ompl::control::LQRPlanner::compute_LQR_plan(
  ompl::base::State * start_state,
  ompl::base::State * goal_state,
  std::vector<ompl::base::State *> & resulting_path)
{
  // double start_to_goal_dist = si_->distance(start_state, goal_state);

  double time = 0.0;
  // double last_dist_to_goal = INFINITY;
  resulting_path.push_back(start_state);

  const auto * start_cstate = start_state->as<ompl::base::ElevationStateSpace::StateType>();
  // const auto * start_so2 = start_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
  const auto * start_xyzv = start_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

  const auto * goal_cstate = goal_state->as<ompl::base::ElevationStateSpace::StateType>();
  // const auto * goal_so2 = goal_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
  const auto * goal_xyzv = goal_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

  std::tuple<Eigen::Matrix2d, Eigen::Vector2d,
    Eigen::Matrix2d, double>
    ABQR = getABQR();
  Eigen::Matrix2d A = std::get<0>(ABQR);
  Eigen::Vector2d B = std::get<1>(ABQR);
  Eigen::Matrix2d Q = std::get<2>(ABQR);
  double R = std::get<3>(ABQR);

  int iter = 0;

  while (time < max_time_) {
    auto * latest_cstate =
      resulting_path.back()->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * latest_so2 = latest_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
    const auto * latest_xyzv = latest_cstate->as<ompl::base::RealVectorStateSpace::StateType>(
      1);

    double xc = latest_xyzv->values[0];
    double yc = latest_xyzv->values[1];
    double thetac = latest_so2->value;

    double theta_r = std::atan2(
      (yc - start_xyzv->values[1]),
      (xc - start_xyzv->values[0]));

    Eigen::MatrixXd T(3, 3);
    T << -std::cos(theta_r), -std::sin(theta_r), 0,
      std::sin(theta_r), -std::cos(theta_r), 0,
      0, 0, 1;

    Eigen::VectorXd e(3);
    e(0) = xc - goal_xyzv->values[0];
    e(1) = yc - goal_xyzv->values[1];
    e(2) = thetac - theta_r;

    Eigen::Vector3d Te_dynamics = T * e;
    Eigen::Vector2d X(Te_dynamics(1), Te_dynamics(2));

    double phi = lqr_control(A, B, Q, R, X);
    phi = std::clamp<double>(phi, -phi_bound_, phi_bound_);

    Eigen::VectorXd U(2);
    U(0) = v_r_;
    U(1) = phi;

    // Propogate the states with computed optimal control3
    // Store the state in the resulting path as that really is
    auto * this_state = si_->allocState();
    auto * this_cstate = this_state->as<ompl::base::ElevationStateSpace::StateType>();

    /*propogate according to car-like dynamics*/
    auto d_to_goal = si_->distance(this_state, goal_state);
    auto d_to_start = si_->distance(this_state, start_state);

    // linear inetrpolation for intermediate z values
    double true_z =
      ((start_xyzv->values[2] * d_to_goal) + (goal_xyzv->values[2] * d_to_start)) /
      (d_to_start + d_to_goal);
    this_cstate->setXYZV(
      xc + dt_ * U(0) * std::cos(thetac),
      yc + dt_ * U(0) * std::sin(thetac),
      true_z,
      U(0));

    this_cstate->setSO2(
      thetac + dt_ * (U(0) * std::tan(U(1)) / L_));

    if (si_->distance(this_state, goal_state) < goal_tolerance_) {
      // if Reached the goal or max time reached, break the loop
      U(0) = 0;
      break;
    }

    resulting_path.push_back(this_state);

    // TIME STEP INCREASE
    time += dt_;

    iter++;
  }

  return resulting_path;
}

// return optimal steering angle
double ompl::control::LQRPlanner::lqr_control(
  const Eigen::Matrix2d & A,
  const Eigen::Vector2d & B,
  const Eigen::Matrix2d & Q,
  const double & R,
  const Eigen::Vector2d & X)
{
  std::tuple<Eigen::Vector2d, Eigen::Matrix2d, Eigen::VectorXcd> res = dlqr(A, B, Q, R);
  Eigen::Vector2d K = std::get<0>(res);
  double phi = (K.transpose() * X).value();
  return phi;
}

/**
 * @brief authored by: Horibe Takamasa; https://github.com/TakaHoribe
 *        Continous time Riccati Eq. solver
 */
bool ompl::control::LQRPlanner::solve_care(
  const Eigen::Matrix2d & A,
  const Eigen::Vector2d & B,
  const Eigen::Matrix2d & Q,
  const double & R,
  Eigen::Matrix2d & P,
  const double dt,
  const double & tolerance,
  const uint iter_max)
{
  P = Q;  // initialize
  Eigen::Matrix2d P_next;
  Eigen::Matrix2d AT = A.transpose();
  double Rinv = 1.0 / R;

  double diff;
  for (uint i = 0; i < iter_max; ++i) {
    P_next = P + (P * A + AT * P - P * B * Rinv * B.transpose() * P + Q) * dt;
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if (diff < tolerance) {
      return true;
    }
  }
  return false;  // over iteration limit
}

/**
 * @brief
 * Solve the discrete time lqr controller.
 * x[k+1] = A x[k] + B u[k]
 * cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
 * ref Bertsekas, p.151

 * @param Ad
 * @param Bd
 * @param Q
 * @param R
 * @return std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXcd>, K, X, EIGENVALUES
 */
std::tuple<Eigen::Vector2d, Eigen::Matrix2d, Eigen::VectorXcd> ompl::control::LQRPlanner::dlqr(
  const Eigen::Matrix2d & /*A*/,
  const Eigen::Vector2d & B,
  const Eigen::Matrix2d & /*Q*/,
  const double & R)
{
  Eigen::Matrix2d P;
  // bool solved_dare = solve_care(A, B, Q, R, P);
  Eigen::Vector2d K = (1 / R) * (B.transpose() * P);
  Eigen::VectorXcd eig /* = (A - B.transpose() * K).eigenvalues()*/;
  return std::make_tuple(K, P, eig);
}
