// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_PLANNING__NATIVE_PLANNERS__COST_TRUST_KINO_PLANNER_HPP_
#define VOX_NAV_PLANNING__NATIVE_PLANNERS__COST_TRUST_KINO_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// OMPL BASE
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/Planner.h>
#include <ompl/base/OptimizationObjective.h>

// OMPL CONTROL 
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/PathControl.h>

// OMPL DATASTUCTURE
#include <ompl/datastructures/NearestNeighbors.h>

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <thread>

namespace ompl
{
namespace control
{
/**
   @anchor cCostTrustKinoPlanner
   @par Short description
   \ref This algorithm features a novel node selection process, identifying 'frontier' nodes based on multiple criteria:
        the number of branches per node, the cost from the start node, and local node density.
        This approach prioritizes nodes that potentially enable quicker exploration of the state space.
        Once frontier nodes are selected, they are expanded using random controls to converge towards the desired goal
   pose. Upon finding a solution, the algorithm randomly selects a segment of the path, resets the tree, and reruns the
   process for continuous optimization. Additionally, the planner operates on a multi-threaded architecture,
   significantly enhancing the probability of discovering an optimal path. An accompanying paper explaining novelities
   of this planner will be published soon.

   @par External documentation
   TBD
*/
struct CostTrustParameters
{
  /** \brief All configurable parameters of CostTrustKinoPlanner. */

  /** \brief The number of threads to be used in parallel for geometric and control. No odd numbers and less than 2 */
  int num_threads_{6};

  /** \brief The minimum number of branches to be extended from one frontier node */
  int min_number_of_branches_to_extend_{5};

  /** \brief The maximum number of branches to be extended from one frontier node */
  int max_number_of_branches_to_extend_{10};

  /** \brief The maximum number of frontier nodes to be selected for extension */
  std::size_t max_number_of_frontier_nodes_{25};

  /** \brief The minimum distance between nodes in the graph */
  double min_distance_between_nodes_{0.1};

  /** \brief The coefficient for determining score of best frontier node to be extended */
  double k_prefer_nodes_with_low_branches_{1.0};

  /** \brief The coefficient for determining score of best frontier node to be extended */
  double k_prefer_nodes_with_high_cost_{5.0};

  /** \brief The coefficient for determining score of best frontier node to be extended */
  int num_of_neighbors_to_consider_for_density_{20};
};

class CostTrustKinoPlanner : public ompl::base::Planner
{
public:
  /** \brief Constructor */
  CostTrustKinoPlanner(const ompl::control::SpaceInformationPtr & si);

  /** \brief Destructor */
  ~CostTrustKinoPlanner() override;

  /** \brief Setup the planner */
  void setup() override;

  /** \brief Continue solving for some amount of time. Return true if solution was found. */
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc) override;

  /** \brief Get the internal data owned by planner, NOT IMPLEMENTED */
  void getPlannerData(ompl::base::PlannerData & data) const override;

  /** \brief Clear datastructures. Call this function if the
      input data to the planner has changed and you do not
      want to continue planning */
  void clear() override;

  /** \brief Free the memory allocated by this planner. That is mostly in nearest neihbours. */
  void freeMemory();

  /** \brief Properties of boost graph vertex, both geometriuc and control graphs share this vertex property.
   *  Some of the elements are not used in geometric graph (e.g., control, control_duration). */
  struct VertexProperty
  {
    ompl::base::State * state{nullptr};
    ompl::control::Control * control{nullptr};
    ompl::base::Cost cost{std::numeric_limits<double>::infinity()};
    ompl::base::Cost cost_to_go{std::numeric_limits<double>::infinity()};
    unsigned int control_duration{0};
    bool blacklisted{false};
    bool is_root{false};
    bool belongs_to_solution{false};
    std::vector<VertexProperty *> branches;
    VertexProperty * parent{nullptr};
  };

  /** \brief Compute distance between Vertexes (actually distance between contained states) */
  double distanceFunction(const VertexProperty * a, const VertexProperty * b) const;

  /** \brief Compute distance between states */
  double distanceFunction(const ompl::base::State * a, const ompl::base::State * b) const;

  /** \brief Given its vertex_descriptor (id),
   * return a const pointer to VertexProperty in control graph g_forward_control_  */
  const VertexProperty * getVertexControls(std::size_t id, int thread_id);

  void setNumThreads(int num_threads);
  int getNumThreads() const;

  void setMinNumberOfBranchesToExtend(int min_number_of_branches_to_extend);
  int getMinNumberOfBranchesToExtend() const;

  void setMaxNumberOfBranchesToExtend(int max_number_of_branches_to_extend);
  int getMaxNumberOfBranchesToExtend() const;

  void setMaxNumberOfFrontierNodes(int max_number_of_frontier_nodes);
  int getMaxNumberOfFrontierNodes() const;

  void setMinDistanceBetweenNodes(double min_distance_between_nodes);
  double getMinDistanceBetweenNodes() const;

  void setKPreferNodesWithLowBranches(double k_prefer_nodes_with_low_branches);
  double getKPreferNodesWithLowBranches() const;

  void setKPreferNodesWithHighCost(double k_prefer_nodes_with_high_cost);
  double getKPreferNodesWithHighCost() const;

  void setNumOfNeighborsToConsiderForDensity(int num_of_neighbors_to_consider_for_density);
  int getNumOfNeighborsToConsiderForDensity() const;

private:
  /** \brief All configurable parames live here. */
  CostTrustParameters params_;

  /** \brief Control space information */
  const ompl::control::SpaceInformation * siC_{nullptr};

  /** \brief The optimization objective. */
  ompl::base::OptimizationObjectivePtr opt_{nullptr};

  /** \brief Current cost of best path. The informed sampling strategy needs it. */
  ompl::base::Cost bestControlCost_{std::numeric_limits<double>::infinity()};

  /** \brief keep the index of best control path, the index comes from thread id*/
  int bestControlPathIndex_{0};

  /** \brief The best path found so far. */
  std::shared_ptr<ompl::control::PathControl> bestControlPath_{nullptr};

  ControlSamplerPtr controlSampler_{nullptr};

  /** \brief The random number generator */
  RNG rng_;

  /** \brief The NN datastructure for control graph */
  std::vector<std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>>> nnControlsThreads_;

  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> bestControlPathNN_{nullptr};

  std::vector<VertexProperty *> startVerticesControl_;
  std::vector<VertexProperty *> goalVerticesControl_;

  void selectExplorativeFrontiers(std::size_t max_number,
    std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & nn_structure,
    std::vector<VertexProperty *> & frontier_nodes);

  void extendFrontiers(std::vector<VertexProperty *> & frontier_nodes, std::size_t num_branch_to_extend,
    std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & nn_structure,
    const ompl::base::PlannerTerminationCondition & ptc, VertexProperty * target_vertex_property,
    std::shared_ptr<PathControl> & path, std::vector<VertexProperty *> & control_paths_vertices,
    const bool exact_solution_found, bool * should_stop_exploration,
    const std::shared_ptr<PathControl> & current_best_path);

  /** \brief compute path cost by finding cost between
   * consecutive vertices in the path
   * \param vertex_path is the path
   */
  ompl::base::Cost computePathCost(std::shared_ptr<ompl::control::PathControl> & path) const;

  /** \brief compute path cost by finding cost between
   * consecutive vertices in the path
   * \param vertex_path is the path
   */
  ompl::base::Cost computePathCost(const std::shared_ptr<ompl::control::PathControl> & path) const;

  /** \brief Extract the states from the path and return them as a vector of const states
   * \param path is the ompl::control::PathControl from which the states are extracted
   */
  std::vector<const ompl::base::State *> getConstStatesFromPath(const std::shared_ptr<ompl::control::PathControl> & path);

  // RVIZ Visualization of planner progess, this will be removed in the future

  /** \brief static method to visulize a graph in RVIZ*/
  static void visualizeRGG(const std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & nn_structure,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
    const std::string & ns, const std_msgs::msg::ColorRGBA & color, const int & state_space_type);

  static void visualizePath(const std::shared_ptr<PathControl> & path,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
    const std::string & ns, const std_msgs::msg::ColorRGBA & color, const int & state_space_type);

  /** \brief get std_msgs::msg::ColorRGBA given the color name with a std::string*/
  static std_msgs::msg::ColorRGBA getColor(std::string & color);

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rgg_graph_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geometric_path_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr first_control_graph_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr second_control_graph_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_path_pub_;

  /** \brief The node*/
  rclcpp::Node::SharedPtr node_;

};  // class CostTrustKinoPlanner
}  // namespace control
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__NATIVE_PLANNERS__COST_TRUST_KINO_PLANNER_HPP_
