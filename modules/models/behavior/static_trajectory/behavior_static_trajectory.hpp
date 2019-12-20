// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_STATIC_TRAJECTORY_BEHAVIOR_STATIC_TRAJECTORY_HPP_
#define MODULES_MODELS_BEHAVIOR_STATIC_TRAJECTORY_BEHAVIOR_STATIC_TRAJECTORY_HPP_

#include <memory>

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::DynamicModelPtr;
using dynamic::Input;
using dynamic::State;
using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;
using StateRowVector = Eigen::Matrix<State::Scalar, 1, Eigen::Dynamic>;

class BehaviorStaticTrajectory : public BehaviorModel {
 public:
  BehaviorStaticTrajectory(commons::Params* params,
                           const Trajectory& static_trajectory);
  Trajectory Plan(float delta_time,
                  const world::ObservedWorld& observed_world) override;
  std::shared_ptr<BehaviorModel> Clone() const override;

 private:

  std::pair<int,int> interpolate(const double t, StateRowVector *interpolated) const;
  Trajectory static_trajectory_;
};

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_STATIC_TRAJECTORY_BEHAVIOR_STATIC_TRAJECTORY_HPP_
