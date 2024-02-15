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

#include "vox_nav_planning/native_planners/RRTStarF.hpp"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/PathControl.h>
#include <ompl/tools/config/SelfConfig.h>

ompl::control::RRTStarF::RRTStarF(const SpaceInformationPtr & si)
: base::Planner(si, "RRTStarF")
{
  specs_.approximateSolutions = true;
  siC_ = si.get();
}

ompl::control::RRTStarF::~RRTStarF()
{
  freeMemory();
}

void ompl::control::RRTStarF::setup()
{
  base::Planner::setup();
  if (!nn_) {
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<ompl::control::RRTStarF::Node *>(this));
  }
  nn_->setDistanceFunction([this](const ompl::control::RRTStarF::Node * a, const ompl::control::RRTStarF::Node * b) { return distanceFunction(a, b); });

  if (pdef_) {
    if (pdef_->hasOptimizationObjective()) {
      opt_ = pdef_->getOptimizationObjective();
    }
    else {
      OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
      opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
      pdef_->setOptimizationObjective(opt_);
    }
  }

  // ros2 node to publish rrt nodes
  node_ = std::make_shared<rclcpp::Node>("rrtstarf_rclcpp_node");

  rrt_nodes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/rrtstar/nodes",
    rclcpp::SystemDefaultsQoS());
}

void ompl::control::RRTStarF::clear()
{
  Planner::clear();
  sampler_.reset();
  valid_state_sampler_.reset();
  freeMemory();
  if (nn_) {
    nn_->clear();
  }
}

void ompl::control::RRTStarF::freeMemory()
{
}

ompl::base::PlannerStatus ompl::control::RRTStarF::solve(const base::PlannerTerminationCondition & ptc)
{
  checkValidity();
  base::Goal * goal = pdef_->getGoal().get();
  auto * goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

  // get the goal node and state
  auto * goal_node = new ompl::control::RRTStarF::Node(siC_);
  while (const base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_node->state_, goal);
  }

  // get start node and state,push  the node inton nn_ as well
  auto * start_node = new ompl::control::RRTStarF::Node(siC_);
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_node->state_, st);
    nn_->add(start_node);
  }

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  // Use valid state sampler
  if (!valid_state_sampler_) {
    valid_state_sampler_ = si_->allocValidStateSampler();
  }
  if (!sampler_) {
    sampler_ = si_->allocStateSampler();
  }

  OMPL_INFORM("%s: Starting planning with %u states already in datastructure\n", getName().c_str(), nn_->size());

  auto * random_node = new Node(siC_);
  base::State * random_state = random_node->state_;

  unsigned iterations = 0;

  Node * last_node = new Node(siC_);
  Node * last_valid_node = new Node(siC_);
  last_valid_node->cost_ = base::Cost(std::numeric_limits<double>::max());

  while (ptc == false) {
    /* sample random state (with goal biasing) */
    if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
      goal_s->sampleGoal(random_state);
    }
    else {
      sampler_->sampleUniform(random_state);
      // valid_state_sampler_->sample(random_state);
    }

    auto nearest_node = get_nearest_node(random_node);
    auto new_node = steer(nearest_node, random_node, expand_dis_);
    auto inc_cost = opt_->motionCost(nearest_node->state_, new_node->state_);
    new_node->cost_ = opt_->combineCosts(nearest_node->cost_, inc_cost);

    if (check_collision(new_node)) {
      std::vector<Node *> near_nodes = find_near_nodes(new_node);
      auto node_with_updated_parent = choose_parent(new_node, near_nodes);
      if (node_with_updated_parent != nullptr) {
        // check motion
        if (!si_->checkMotion(node_with_updated_parent->state_, new_node->state_)) {
          continue;
        }
        rewire(node_with_updated_parent, near_nodes);
        nn_->add(node_with_updated_parent);
      }
      else {
        nn_->add(new_node);
      }
      last_node = search_best_goal_node(goal_node);
    }

    if (last_node != nullptr && last_node->cost_.value() < last_valid_node->cost_.value()) {
      last_valid_node = last_node;
    }
    iterations++;

    if (iterations % 200 == 0 && last_valid_node && static_cast<int>(last_valid_node->cost_.value())) {
      OMPL_INFORM("Current solution cost %.2f", last_valid_node->cost_.value());
    }
  }

  auto final_node = steer(last_valid_node, goal_node);
  if (final_node) {
    if (check_collision(final_node)) {
      std::vector<Node *> near_nodes = find_near_nodes(final_node);
      auto final_node_parent = choose_parent(final_node, near_nodes);
      if (final_node_parent != nullptr) {
        nn_->add(final_node_parent);
      }
    }
  }

  OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(), iterations);

  // visualize rrt node tree growth in RVIZ
  std::vector<Node *> all_nodes;
  nn_->list(all_nodes);
  visualization_msgs::msg::MarkerArray rrt_nodes;
  int node_index_counter = 0;
  for (auto i : all_nodes) {
    if (i) {
      if (!i->parent_) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "rrt_nodes";
      marker.id = node_index_counter;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker.text = std::to_string(node_index_counter);
      marker.scale.x = 0.08;
      marker.color.a = 1.0;
      marker.color.b = 1.0;
      marker.colors.push_back(marker.color);
      const auto * cstate = i->state_->as<ompl::base::ElevationStateSpace::StateType>();
      // const auto* so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      geometry_msgs::msg::Point node_point, parent_point;
      node_point.x = xyzv->values[0];
      node_point.y = xyzv->values[1];
      node_point.z = xyzv->values[2];
      const auto * parent_cstate = i->parent_->state_->as<ompl::base::ElevationStateSpace::StateType>();
      // const auto* parent_so2 = parent_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * parent_xyzv = parent_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      parent_point.x = parent_xyzv->values[0];
      parent_point.y = parent_xyzv->values[1];
      parent_point.z = parent_xyzv->values[2];
      marker.points.push_back(parent_point);
      marker.points.push_back(node_point);
      rrt_nodes.markers.push_back(marker);
      node_index_counter++;
    }
  }
  rrt_nodes_pub_->publish(rrt_nodes);

  bool solved = false;
  bool approximate = false;
  double approxdif = std::numeric_limits<double>::infinity();

  if (final_node) {
    OMPL_INFORM("Final solution cost %.2f", final_node->cost_.value());
    std::vector<base::State *> final_course = generate_final_course(last_valid_node);
    OMPL_INFORM("Total states in solution %ld", final_course.size());
    final_course = remove_duplicate_states(final_course);
    OMPL_INFORM("Total states in solution after removing duplicates %ld", final_course.size());

    std::reverse(final_course.begin(), final_course.end());

    double dist = 0.0;
    bool solv = goal->isSatisfied(final_course.back(), &dist);
    /* set the solution path */
    auto path(std::make_shared<PathControl>(si_));
    for (auto i : final_course) {
      if (i) {
        path->append(i);
      }
    }

    OMPL_INFORM("Solution Lenght %.2f", path->length());
    OMPL_INFORM("Distance of solution to goal %.2f", dist);

    approxdif = dist;

    if (solv) {
      solved = true;
      approximate = false;
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
      OMPL_INFORM("Found solution with cost %.2f", last_valid_node->cost_.value());
    }
    else if (!solv && (path->length() > 1.0)) {  // approx
      solved = true;
      approximate = true;
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
      OMPL_INFORM("%s: Approx solution with cost %.2f", getName().c_str(), last_valid_node->cost_.value());
    }
    else {
      solved = false;
      approximate = false;
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
      OMPL_WARN("%s: Failed to cretae a plan", getName().c_str());
    }
  }

  clear();

  return {solved, approximate};
}

void ompl::control::RRTStarF::getPlannerData(base::PlannerData & data) const
{
  Planner::getPlannerData(data);

  std::vector<Node *> Nodes;
  std::vector<Node *> allNodes;
  if (nn_) {
    nn_->list(Nodes);
  }

  for (unsigned i = 0; i < allNodes.size(); i++) {
    if (allNodes[i]->parent_ != nullptr) {
      allNodes.push_back(allNodes[i]->parent_);
    }
  }

  // double delta = siC_->getPropagationStepSize();

  for (auto m : allNodes) {
    if (m->parent_) {
      data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_));
    }
    else {
      data.addStartVertex(base::PlannerDataVertex(m->state_));
    }
  }
}

ompl::control::RRTStarF::Node * ompl::control::RRTStarF::steer(ompl::control::RRTStarF::Node * from_node, ompl::control::RRTStarF::Node * to_node, float extend_length)
{
  auto * new_node = new Node(siC_);
  new_node->state_ = si_->allocState();
  si_->copyState(new_node->state_, from_node->state_);

  auto d_theta_beta = calc_distance_and_angle(from_node, to_node);
  auto d = std::get<0>(d_theta_beta);
  auto theta = std::get<1>(d_theta_beta);
  auto beta = std::get<2>(d_theta_beta);

  new_node->path_.push_back(new_node->state_);

  if (extend_length > d) {
    extend_length = d;
  }

  int n_expand = static_cast<int>(std::floor(extend_length / path_resolution_));

  for (int i = 0; i < n_expand; i++) {
    auto * new_node_cstate = new_node->state_->as<ompl::base::ElevationStateSpace::StateType>();

    double x = new_node_cstate->getXYZV()->values[0] + path_resolution_ * std::cos(theta);
    double y = new_node_cstate->getXYZV()->values[1] + path_resolution_ * std::sin(theta);
    double z = new_node_cstate->getXYZV()->values[2] + path_resolution_ * std::sin(beta);
    new_node_cstate->setXYZV(x, y, z, 0);
    new_node_cstate->setSO2(0);
    new_node->path_.push_back(new_node->state_);
  }

  d_theta_beta = calc_distance_and_angle(new_node, to_node);
  d = std::get<0>(d_theta_beta);
  theta = std::get<1>(d_theta_beta);
  beta = std::get<2>(d_theta_beta);

  if (d <= path_resolution_) {
    auto * new_node_cstate = new_node->state_->as<ompl::base::ElevationStateSpace::StateType>();
    auto * to_node_cstate = to_node->state_->as<ompl::base::ElevationStateSpace::StateType>();

    new_node->path_.push_back(to_node->state_);
    new_node_cstate->setXYZV(to_node_cstate->getXYZV()->values[0], to_node_cstate->getXYZV()->values[1],
      to_node_cstate->getXYZV()->values[2], to_node_cstate->getXYZV()->values[3]);
    new_node_cstate->setSO2(to_node_cstate->getSO2()->value);
  }

  new_node->parent_ = from_node;
  return new_node;
}

std::tuple<double, double, double> ompl::control::RRTStarF::calc_distance_and_angle(ompl::control::RRTStarF::Node * from_node, ompl::control::RRTStarF::Node * to_node)
{
  auto * from_node_cstate = from_node->state_->as<base::ElevationStateSpace::StateType>();
  auto * to_node_cstate = to_node->state_->as<base::ElevationStateSpace::StateType>();

  double dx = to_node_cstate->getXYZV()->values[0] - from_node_cstate->getXYZV()->values[0];
  double dy = to_node_cstate->getXYZV()->values[1] - from_node_cstate->getXYZV()->values[1];
  double dz = to_node_cstate->getXYZV()->values[2] - from_node_cstate->getXYZV()->values[2];

  double d = std::hypot(dx, dy, dz);
  double theta = std::atan2(dy, dx);
  double beta = std::atan2(dz, std::hypot(dx, dy));

  return std::make_tuple(d, theta, beta);
}

ompl::control::RRTStarF::Node * ompl::control::RRTStarF::get_nearest_node(ompl::control::RRTStarF::Node * rnd)
{
  Node * nearest_node = nn_->nearest(rnd);
  return nearest_node;
}

bool ompl::control::RRTStarF::check_collision(ompl::control::RRTStarF::Node * new_node)
{
  for (auto i : new_node->path_) {
    if (!si_->isValid(i)) {
      return false;
    }
  }
  return true;
}

std::vector<ompl::control::RRTStarF::Node *> ompl::control::RRTStarF::find_near_nodes(ompl::control::RRTStarF::Node * new_node)
{
  std::vector<Node *> near_nodes;
  auto nnode = nn_->size() + 1;
  double r = connect_circle_dist_ * std::sqrt((std::log(nnode) / nnode));
  r = std::min(r, expand_dis_);
  nn_->nearestR(new_node, r * r, near_nodes);
  return near_nodes;
}

double ompl::control::RRTStarF::calc_new_cost(ompl::control::RRTStarF::Node * from_node, ompl::control::RRTStarF::Node * to_node)
{
  base::Cost relative_cost = opt_->motionCost(from_node->state_, to_node->state_);
  return opt_->combineCosts(from_node->cost_, relative_cost).value();
}

ompl::control::RRTStarF::Node * ompl::control::RRTStarF::choose_parent(ompl::control::RRTStarF::Node * new_node, std::vector<ompl::control::RRTStarF::Node *> near_nodes)
{
  if (!near_nodes.size()) {
    return nullptr;
  }
  std::vector<double> costs;
  for (auto near_node : near_nodes) {
    Node * t_node = steer(near_node, new_node);
    if (t_node && check_collision(t_node)) {
      costs.push_back(calc_new_cost(near_node, new_node));
    }
    else {
      costs.push_back(INFINITY);
    }
  }
  double min_cost = *std::min_element(costs.begin(), costs.end());
  int min_cost_index = std::min_element(costs.begin(), costs.end()) - costs.begin();

  if (min_cost == INFINITY) {
    std::cerr << "There is no good path.(min_cost is inf)" << std::endl;
    return nullptr;
  }
  new_node = steer(near_nodes[min_cost_index], new_node);
  new_node->cost_ = base::Cost(min_cost);
  return new_node;
}

void ompl::control::RRTStarF::rewire(ompl::control::RRTStarF::Node * new_node, std::vector<ompl::control::RRTStarF::Node *> near_nodes)
{
  for (auto near_node : near_nodes) {
    Node * edge_node = steer(new_node, near_node);
    if (!edge_node) {
      continue;
    }

    edge_node->cost_ = base::Cost(calc_new_cost(new_node, near_node));
    bool no_collision = check_collision(edge_node);
    bool improved_cost = near_node->cost_.value() > edge_node->cost_.value();

    if (no_collision && improved_cost) {
      si_->copyState(near_node->state_, edge_node->state_);

      near_node->cost_ = edge_node->cost_;
      near_node->path_ = edge_node->path_;
      near_node->parent_ = edge_node->parent_;
      propagate_cost_to_leaves(new_node);
    }
  }
}

void ompl::control::RRTStarF::propagate_cost_to_leaves(ompl::control::RRTStarF::Node * parent_node)
{
  if (nn_) {
    std::vector<Node *> nodes;
    nn_->list(nodes);
    for (auto & node : nodes) {
      if (node == parent_node) {
        node->cost_ = base::Cost(calc_new_cost(parent_node, node));
        propagate_cost_to_leaves(node);
      }
    }
  }
}

double ompl::control::RRTStarF::calc_dist_to_goal(ompl::control::RRTStarF::Node * /*node*/, ompl::base::Goal * /*goal*/)
{
  double dist = 0.0;
  // bool solv = goal->isSatisfied(node->state_, &dist);
  return dist;
}

ompl::control::RRTStarF::Node * ompl::control::RRTStarF::search_best_goal_node(ompl::control::RRTStarF::Node * goal_node)
{
  std::vector<Node *> near_nodes;
  nn_->nearestR(goal_node, expand_dis_, near_nodes);
  base::Cost bestCost = opt_->infiniteCost();
  Node * selected = nullptr;

  for (auto & i : near_nodes) {
    if (opt_->isCostBetterThan(i->cost_, bestCost)) {
      selected = i;
      bestCost = i->cost_;
    }
  }
  return selected;
}

double ompl::control::RRTStarF::distanceFunction(const ompl::control::RRTStarF::Node * a, const ompl::control::RRTStarF::Node * b) const
{
  return si_->distance(a->state_, b->state_);
}

double ompl::control::RRTStarF::distanceFunction(const ompl::base::State * a, const ompl::base::State * b) const
{
  return si_->distance(a, b);
}

std::vector<ompl::base::State *> ompl::control::RRTStarF::generate_final_course(ompl::control::RRTStarF::Node * goal_node)
{
  std::vector<ompl::base::State *> final_path;
  final_path.push_back(goal_node->state_);
  auto node = goal_node;

  while (node->parent_) {
    for (auto i : node->path_) {
      final_path.push_back(i);
    }
    node = node->parent_;
  }

  final_path.push_back(node->state_);
  return final_path;
}

std::vector<ompl::base::State *> ompl::control::RRTStarF::remove_duplicate_states(std::vector<ompl::base::State *> all_states)
{
multiCont: /* Empty statement using the semicolon */;

  for (std::size_t i = 0; i < all_states.size(); i++) {
    for (std::size_t j = 0; j < all_states.size(); j++) {
      if ((i != j) && (distanceFunction(all_states[i], all_states[j]) < 0.05)) {
        // j is duplicate
        all_states.erase(all_states.begin() + j);
        goto multiCont;
      }
    }
  }

  std::vector<ompl::base::State *> sorted;
  sorted.push_back(all_states.front());
  all_states.erase(all_states.begin());

sortCont: /* Empty statement using the semicolon */;

  for (std::size_t i = 0; i < all_states.size(); i++) {
    std::size_t closest_idx = 100000;
    double currnet_min = 100000.0;
    all_states[i] = sorted.back();

    for (std::size_t j = 0; j < all_states.size(); j++) {
      double dist = distanceFunction(all_states[i], all_states[j]);
      if (dist < currnet_min && (i != j)) {
        currnet_min = dist;
        closest_idx = j;
      }
    }

    if (closest_idx > all_states.size() - 1) {
      sorted.push_back(all_states.back());
      break;
    }

    sorted.push_back(all_states[closest_idx]);
    all_states.erase(all_states.begin() + closest_idx);
    goto sortCont;
  }

  return sorted;
}
