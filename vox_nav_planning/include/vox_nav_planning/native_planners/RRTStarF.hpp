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

#ifndef VOX_NAV_PLANNING__RRT__RRTSTARF_HPP_
#define VOX_NAV_PLANNING__RRT__RRTSTARF_HPP_

#include <vox_nav_utilities/elevation_state_space.hpp>

#include <nav_msgs/msg/path.hpp>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <limits>
#include <memory>
#include <tuple>
#include <vector>

namespace ompl
{
namespace control
{

class RRTStarF : public base::Planner
{
public:
  /** \brief Constructor */
  RRTStarF(const SpaceInformationPtr & si);

  ~RRTStarF() override;

  void setup() override;

  /** \brief Continue solving for some amount of time. Return true if solution was found. */
  base::PlannerStatus solve(const base::PlannerTerminationCondition & ptc) override;

  void getPlannerData(base::PlannerData & data) const override;

  /** \brief Clear datastructures. Call this function if the
      input data to the planner has changed and you do not
      want to continue planning */
  void clear() override;

  /** \brief Set a different nearest neighbors datastructure */
  template <template <typename T> class NN>
  void setNearestNeighbors()
  {
    if (nn_ && nn_->size() != 0) {
      OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    }
    clear();
    nn_ = std::make_shared<NN<Node *>>();
    setup();
  }

  /** \brief Representation of a motion

      This only contains pointers to parent motions as we
      only need to go backwards in the tree. */
  class Node
  {
  public:
    Node() = default;

    /** \brief Constructor that allocates memory for the state and the control */
    Node(const SpaceInformation * si)
    : state_(si->allocState())
    {
    }

    virtual ~Node() = default;

    virtual base::State * getState() const
    {
      return state_;
    }
    virtual Node * getParent() const
    {
      return parent_;
    }

    base::Cost cost_{0};

    /** \brief The state contained by the motion */
    base::State * state_{nullptr};

    /** \brief The parent motion in the exploration tree */
    Node * parent_{nullptr};

    std::vector<base::State *> path_;
  };

  Node * steer(Node * from_node, Node * to_node, float extend_length = INFINITY);

  std::tuple<double, double, double> calc_distance_and_angle(Node * from_node, Node * to_node);

  Node * get_nearest_node(Node * rnd);

  bool check_collision(Node * new_node);

  std::vector<Node *> find_near_nodes(Node * new_node);

  double calc_new_cost(Node * from_node, Node * to_node);

  Node * choose_parent(Node * new_node, std::vector<Node *> near_nodes);

  void rewire(Node * new_node, std::vector<Node *> near_nodes);

  void propagate_cost_to_leaves(Node * parent_node);

  double calc_dist_to_goal(Node * /*node*/, ompl::base::Goal * /*goal*/);

  Node * search_best_goal_node(Node * goal_node);

  double distanceFunction(const Node * a, const Node * b) const;

  double distanceFunction(const base::State * a, const base::State * b) const;

  std::vector<base::State *> generate_final_course(Node * goal_node);

  std::vector<base::State *> remove_duplicate_states(std::vector<base::State *> all_states);

  /** \brief Free the memory allocated by this planner */
  void freeMemory();

  /** \brief State sampler */
  ompl::base::StateSamplerPtr sampler_;

  const SpaceInformation * siC_;

  /** \brief State sampler */
  ompl::base::ValidStateSamplerPtr valid_state_sampler_;

  /** \brief A nearest-neighbors datastructure containing the tree of motions */
  std::shared_ptr<ompl::NearestNeighbors<Node *>> nn_;

  /** \brief The optimization objective. */
  ompl::base::OptimizationObjectivePtr opt_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_nodes_pub_;

  rclcpp::Node::SharedPtr node_;

  double goalBias_{0.05};
  double path_resolution_{0.015};
  double connect_circle_dist_{100.0};
  double expand_dis_{1.0};

  /** \brief The random number generator */
  RNG rng_;
};
}  // namespace control
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__RRT__RRTSTARF_HPP_
