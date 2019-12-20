// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "behavior_static_trajectory.hpp"

#include <memory>

#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::StateDefinition;

BehaviorStaticTrajectory::BehaviorStaticTrajectory(
    commons::Params *params, const Trajectory &static_trajectory)
    : BehaviorModel(params), static_trajectory_(static_trajectory) {}

Trajectory BehaviorStaticTrajectory::Plan(
    float delta_time, const modules::world::ObservedWorld &observed_world) {
  const double start_time = observed_world.get_world_time();
  const double end_time = start_time + delta_time;
  StateRowVector interp_start;
  StateRowVector interp_end;
  int idx_start = interpolate(start_time, &interp_start).second;
  int idx_end = interpolate(end_time, &interp_end).first;
  if (idx_start < 0 || idx_end < 0) {
    return dynamic::Trajectory();
  }
  const int num_rows = idx_end - idx_start + 1;
  dynamic::Trajectory traj(
      num_rows + 2, static_cast<int>(dynamic::StateDefinition::MIN_STATE_SIZE));
  traj.row(0) = interp_start;
  traj.row(traj.rows() - 1) = interp_end;
  traj.block(1, 0, num_rows, traj.cols()) =
      static_trajectory_.block(idx_start, 0, num_rows, traj.cols());
  return traj;
}
std::pair<int, int> BehaviorStaticTrajectory::interpolate(
    const double t, StateRowVector *interpolated) const {
  StateRowVector delta;
  double alpha;
  int idx = -1;
  assert(static_trajectory_.rows() > 1);
  for (int i = 0; i < static_trajectory_.rows() - 1; ++i) {
    float t_i = static_trajectory_(i, dynamic::TIME_POSITION);
    float t_i_succ = static_trajectory_(i + 1, dynamic::TIME_POSITION);
    if (t_i <= t && t <= t_i_succ) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    return {-1, -1};
  }
  delta = static_trajectory_.row(idx + 1) - static_trajectory_.row(idx);
  alpha = (t - static_trajectory_(idx, dynamic::TIME_POSITION)) /
          delta(dynamic::TIME_POSITION);
  *interpolated = (static_trajectory_.row(idx) + alpha * delta);
  // Index of next valid entry
  if (alpha == 0.0) {
    return {idx - 1, idx + 1};
  } else if (alpha == 1.0) {
    return {idx, idx + 2};
  } else {
    return {idx, idx + 1};
  }
}
std::shared_ptr<BehaviorModel> BehaviorStaticTrajectory::Clone() const {
  std::shared_ptr<BehaviorStaticTrajectory> model_ptr =
      std::make_shared<BehaviorStaticTrajectory>(*this);
  return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
