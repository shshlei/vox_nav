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

#include "vox_nav_utilities/elevation_state_space.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/planner_helpers.hpp"

// OMPL BASE
#include <ompl/base/Cost.h>
#include <ompl/base/SpaceInformation.h>

ompl::base::ElevationStateSpace::ElevationStateSpace(const SE2StateType & state_type, double turningRadius, bool isSymmetric)
: se2_state_type_(state_type),
  rho_(turningRadius),
  isSymmetric_(isSymmetric)
{
  setName("ElevationStateSpace" + getName());
  registerDefaultProjection(std::make_shared<ElevationStateSpaceProjection>(this));
  registerProjection("ElevationStateSpaceProjection", std::make_shared<ElevationStateSpaceProjection>(this));

  type_ = STATE_SPACE_SE2;  // Well, not exactly, but this is the closest, Infromed sampling requirement
  addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
  addSubspace(std::make_shared<RealVectorStateSpace>(4), 1.0);  // x, y, z, v(linear speed)
  lock();

  se2_ = std::make_shared<ompl::base::SE2StateSpace>();
  dubins_ = std::make_shared<ompl::base::DubinsStateSpace>(rho_, isSymmetric_);
  reeds_sheep_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(rho_);
  so2_ = std::make_shared<ompl::base::SO2StateSpace>();
  real_vector_ = std::make_shared<ompl::base::RealVectorStateSpace>(4);  // x,y,z, v(linear speed)

  state1_se2_ = se2_->allocState();
  state2_se2_ = se2_->allocState();
  interpolation_state1_se2_ = se2_->allocState();
  interpolation_state2_se2_ = se2_->allocState();
  interpolated_state_se2_ = se2_->allocState();
}

ompl::base::ElevationStateSpace::~ElevationStateSpace()
{
   se2_->freeState(state1_se2_);
   se2_->freeState(state2_se2_);
   se2_->freeState(interpolation_state1_se2_);
   se2_->freeState(interpolation_state2_se2_);
   se2_->freeState(interpolated_state_se2_);
}

void ompl::base::ElevationStateSpace::setBounds(const ompl::base::RealVectorBounds & se2_bounds, const ompl::base::RealVectorBounds & z_bounds, const ompl::base::RealVectorBounds & v_bounds)
{
  auto xyzv_bounds = std::make_shared<ompl::base::RealVectorBounds>(4);
  xyzv_bounds->setLow(0, se2_bounds.low[0]);    // x-
  xyzv_bounds->setHigh(0, se2_bounds.high[0]);  // x+
  xyzv_bounds->setLow(1, se2_bounds.low[1]);    // y-
  xyzv_bounds->setHigh(1, se2_bounds.high[1]);  // y+
  xyzv_bounds->setLow(2, z_bounds.low[0]);      // z-
  xyzv_bounds->setHigh(2, z_bounds.high[0]);    // z+
  xyzv_bounds->setLow(3, v_bounds.low[0]);      // v-
  xyzv_bounds->setHigh(3, v_bounds.high[0]);    // v+

  as<RealVectorStateSpace>(1)->setBounds(*xyzv_bounds);

  se2_->setBounds(se2_bounds);
  dubins_->setBounds(se2_bounds);
  reeds_sheep_->setBounds(se2_bounds);
  real_vector_->setBounds(*xyzv_bounds);
}

void ompl::base::ElevationStateSpace::enforceBounds(State * state) const
{
  auto * xyzv = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);
  real_vector_->enforceBounds(xyzv);
}

const ompl::base::RealVectorBounds ompl::base::ElevationStateSpace::getBounds() const
{
  auto merged_bounds = std::make_shared<ompl::base::RealVectorBounds>(5);
  merged_bounds->setLow(0, -M_PI);
  merged_bounds->setHigh(0, M_PI);
  merged_bounds->setLow(1, as<RealVectorStateSpace>(1)->getBounds().low[0]);
  merged_bounds->setHigh(1, as<RealVectorStateSpace>(1)->getBounds().high[0]);
  merged_bounds->setLow(2, as<RealVectorStateSpace>(1)->getBounds().low[1]);
  merged_bounds->setHigh(2, as<RealVectorStateSpace>(1)->getBounds().high[1]);
  merged_bounds->setLow(3, as<RealVectorStateSpace>(1)->getBounds().low[2]);
  merged_bounds->setHigh(3, as<RealVectorStateSpace>(1)->getBounds().high[2]);
  merged_bounds->setLow(4, as<RealVectorStateSpace>(1)->getBounds().low[3]);
  merged_bounds->setHigh(4, as<RealVectorStateSpace>(1)->getBounds().high[3]);
  return *merged_bounds;
}

ompl::base::State * ompl::base::ElevationStateSpace::allocState() const
{
  auto * state = new StateType();
  allocStateComponents(state);
  return state;
}

void ompl::base::ElevationStateSpace::freeState(State * state) const
{
  CompoundStateSpace::freeState(state);
}

double ompl::base::ElevationStateSpace::distance(const State * state1, const State * state2) const
{
  const auto * state1_so2 = state1->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * state1_xyzv = state1->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  const auto * state2_so2 = state2->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * state2_xyzv = state2->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  state1_se2_->as<SE2StateSpace::StateType>()->setXY(state1_xyzv->values[0], state1_xyzv->values[1]);
  state1_se2_->as<SE2StateSpace::StateType>()->setYaw(state1_so2->value);

  state2_se2_->as<SE2StateSpace::StateType>()->setXY(state2_xyzv->values[0], state2_xyzv->values[1]);
  state2_se2_->as<SE2StateSpace::StateType>()->setYaw(state2_so2->value);

  if (se2_state_type_ == SE2StateType::SE2) {
    return se2_->distance(state1_se2_, state2_se2_) + std::sqrt(std::pow(state1_xyzv->values[2] - state2_xyzv->values[2], 2));
  }
  if (se2_state_type_ == SE2StateType::DUBINS) {
    return dubins_->distance(state1_se2_, state2_se2_) + std::sqrt(std::pow(state1_xyzv->values[2] - state2_xyzv->values[2], 2));
  }
  return reeds_sheep_->distance(state1_se2_, state2_se2_) + std::sqrt(std::pow(state1_xyzv->values[2] - state2_xyzv->values[2], 2));
}

void ompl::base::ElevationStateSpace::interpolate(const State * from, const State * to, double t, State * state) const
{
  const auto * from_so2 = from->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * from_xyzv = from->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  const auto * to_so2 = to->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * to_xyzv = to->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  auto * interpolated_so2 = state->as<StateType>()->as<SO2StateSpace::StateType>(0);
  auto * interpolated_xyzv = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  interpolation_state1_se2_->as<SE2StateSpace::StateType>()->setXY(from_xyzv->values[0], from_xyzv->values[1]);
  interpolation_state1_se2_->as<SE2StateSpace::StateType>()->setYaw(from_so2->value);

  interpolation_state2_se2_->as<SE2StateSpace::StateType>()->setXY(to_xyzv->values[0], to_xyzv->values[1]);
  interpolation_state2_se2_->as<SE2StateSpace::StateType>()->setYaw(to_so2->value);

  if (se2_state_type_ == SE2StateType::SE2) {
    se2_->interpolate(interpolation_state1_se2_, interpolation_state2_se2_, t, interpolated_state_se2_);
  }
  else if (se2_state_type_ == SE2StateType::DUBINS) {
    dubins_->interpolate(interpolation_state1_se2_, interpolation_state2_se2_, t, interpolated_state_se2_);
  }
  else {
    reeds_sheep_->interpolate(interpolation_state1_se2_, interpolation_state2_se2_, t, interpolated_state_se2_);
  }

  interpolated_so2->value = interpolated_state_se2_->as<SE2StateSpace::StateType>()->getYaw();     // so2
  interpolated_xyzv->values[0] = interpolated_state_se2_->as<SE2StateSpace::StateType>()->getX();  // x
  interpolated_xyzv->values[1] = interpolated_state_se2_->as<SE2StateSpace::StateType>()->getY();  // y
  interpolated_xyzv->values[2] = from_xyzv->values[2] + t * (to_xyzv->values[2] - from_xyzv->values[2]);                // z
  interpolated_xyzv->values[3] = from_xyzv->values[3] + t * (to_xyzv->values[3] - from_xyzv->values[3]);                // v
}

void ompl::base::ElevationStateSpace::printState(const State * state, std::ostream & out) const
{
  auto * so2 = state->as<StateType>()->as<SO2StateSpace::StateType>(0);
  auto * xyzv = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);
  so2_->printState(so2, out);
  real_vector_->printState(xyzv, out);
}
