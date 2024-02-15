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

#ifndef VOX_NAV_PLANNING__RRT__LQRPLANNER_HPP_
#define VOX_NAV_PLANNING__RRT__LQRPLANNER_HPP_

#include <ompl/base/State.h>
#include <ompl/base/Planner.h>
#include <ompl/control/SpaceInformation.h>

#include <tuple>
#include <vector>

namespace ompl
{
namespace control
{

class LQRPlanner : public ompl::base::Planner
{
public:
  /** \brief Plan from start to pose by LQR for linearized car model
   *  https://ieeexplore.ieee.org/document/7553742
   */
  LQRPlanner(const SpaceInformationPtr & si);

  ~LQRPlanner() override;

  void setup() override;

  /** \brief Continue solving for some amount of time. Return true if solution was found. */
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc) override;

  void getPlannerData(ompl::base::PlannerData & data) const override;

  /** \brief Clear datastructures. Call this function if the
      input data to the planner has changed and you do not
      want to continue planning */
  void clear() override;

  /** \brief Free the memory allocated by this planner */
  void freeMemory();

  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_nodes_pub_;

  // rclcpp::Node::SharedPtr node_;

  double dt_{0.25};
  double max_time_{2.0};
  double q1_{1};
  double q2_{10};
  double r_{1};
  double v_r_{1.5};
  double L_{0.8};
  double phi_bound_{0.4};
  double goal_tolerance_{0.5};

  void set_v(double v)
  {
    v_r_ = v;
  }

  void set_dt(double dt)
  {
    dt_ = dt;
  }

  void set_phi_bound(double phi_bound)
  {
    phi_bound_ = phi_bound;
  }

  void set_max_time(double max_time)
  {
    max_time_ = max_time;
  }

  void set_goal_tolerance(double goal_tolerance)
  {
    goal_tolerance_ = goal_tolerance;
  }

  void update_params(double dt, double max_time, double q1, double q2, double r, double v_r, double L, double phi_bound, double goal_tolerance)
  {
    dt_ = dt;
    max_time_ = max_time;
    q1_ = q1;
    q2_ = q2;
    r_ = r;
    v_r_ = v_r;
    L_ = L;
    phi_bound_ = phi_bound;
    goal_tolerance_ = goal_tolerance;
  }

  std::tuple<Eigen::Matrix2d, Eigen::Vector2d, Eigen::Matrix2d, double> getABQR();

  std::vector<ompl::base::State *> compute_LQR_plan(
    ompl::base::State * start_state,
    ompl::base::State * goal_state,
    std::vector<ompl::base::State *> & resulting_path);

  // return optimal steering angle
  double lqr_control(
    const Eigen::Matrix2d & A,
    const Eigen::Vector2d & B,
    const Eigen::Matrix2d & Q,
    const double & R,
    const Eigen::Vector2d & X);

  /**
   * @brief authored by: Horibe Takamasa; https://github.com/TakaHoribe
   *        Continous time Riccati Eq. solver
   */
  bool solve_care(
    const Eigen::Matrix2d & A,
    const Eigen::Vector2d & B,
    const Eigen::Matrix2d & Q,
    const double & R,
    Eigen::Matrix2d & P,
    const double dt = 0.001,
    const double & tolerance = 1.E-5,
    const uint iter_max = 100000);

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
  std::tuple<Eigen::Vector2d, Eigen::Matrix2d, Eigen::VectorXcd> dlqr(
    const Eigen::Matrix2d & /*A*/,
    const Eigen::Vector2d & B,
    const Eigen::Matrix2d & /*Q*/,
    const double & R);
};

}  // namespace control
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__RRT__LQRPLANNER_HPP_
