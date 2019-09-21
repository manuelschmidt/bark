// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::StateDefinition;

BehaviorMotionPrimitives::BehaviorMotionPrimitives(const DynamicModelPtr& dynamic_model, commons::Params *params) : 
    BehaviorModel(params),
    dynamic_model_(dynamic_model),
    motion_primitives_(),
    active_motion_(),
    integration_time_delta_(params->get_real("BehaviorMotionPrimitives::integration_time_delta",
                                             "the size of the time steps used within the euler integration loop", 0.001)),
    use_frenet_motions_(params->get_bool("BehaviorMotionPrimitives::use_frenet_motions",
          "If true motion primitives inputs are interpreted as longitudinal acceleration and lateral velocity.", false))
    {}

dynamic::Trajectory BehaviorMotionPrimitives::Plan(float delta_time,
    const world::ObservedWorld& observed_world) {
      if(use_frenet_motions_) {
        return PlanFrenet(delta_time, observed_world);
      } else {
        return PlanContinuous(delta_time, observed_world);
      }
    }

dynamic::Trajectory BehaviorMotionPrimitives::PlanContinuous(
    float delta_time,
    const world::ObservedWorld& observed_world) {
  dynamic::State ego_vehicle_state = observed_world.current_ego_state();
  double start_time = observed_world.get_world_time();
   const float dt = integration_time_delta_; 
  const int num_trajectory_points = static_cast<int>(std::floor(delta_time / dt));

  dynamic::Trajectory traj(num_trajectory_points, int(dynamic::StateDefinition::MIN_STATE_SIZE));

  
  int trajectory_idx=0;
  // set first state as start of trajectory
  traj(trajectory_idx, StateDefinition::TIME_POSITION) = start_time;
  traj(trajectory_idx, StateDefinition::X_POSITION) = ego_vehicle_state(StateDefinition::X_POSITION);
  traj(trajectory_idx, StateDefinition::Y_POSITION) = ego_vehicle_state(StateDefinition::Y_POSITION);
  traj(trajectory_idx, StateDefinition::THETA_POSITION) = ego_vehicle_state(StateDefinition::THETA_POSITION);
  traj(trajectory_idx, StateDefinition::VEL_POSITION) = ego_vehicle_state(StateDefinition::VEL_POSITION); 
  
  auto old_state = ego_vehicle_state; // only for getting type, todo: improve
  for (++trajectory_idx; trajectory_idx<num_trajectory_points; ++trajectory_idx) {
    old_state(StateDefinition::TIME_POSITION) = traj(trajectory_idx-1, StateDefinition::TIME_POSITION);
    old_state(StateDefinition::X_POSITION) = traj(trajectory_idx-1, StateDefinition::X_POSITION);
    old_state(StateDefinition::Y_POSITION) = traj(trajectory_idx-1, StateDefinition::Y_POSITION);
    old_state(StateDefinition::THETA_POSITION) = traj(trajectory_idx-1, StateDefinition::THETA_POSITION);
    old_state(StateDefinition::VEL_POSITION) = traj(trajectory_idx-1, StateDefinition::VEL_POSITION);

    float integration_time = dt;
    if(trajectory_idx == num_trajectory_points-1) {
      integration_time = delta_time - trajectory_idx*dt;
      BARK_EXPECT_TRUE(integration_time > 0);
    }
    
    auto state = dynamic::euler_int(*dynamic_model_, old_state, motion_primitives_[active_motion_], integration_time);
    traj(trajectory_idx, StateDefinition::TIME_POSITION) = start_time + trajectory_idx*dt;
    traj(trajectory_idx, StateDefinition::X_POSITION) = state(StateDefinition::X_POSITION);
    traj(trajectory_idx, StateDefinition::Y_POSITION) = state(StateDefinition::Y_POSITION);
    traj(trajectory_idx, StateDefinition::THETA_POSITION) = state(StateDefinition::THETA_POSITION);
    traj(trajectory_idx, StateDefinition::VEL_POSITION) = state(StateDefinition::VEL_POSITION);
  }

  set_last_action(Action(DiscreteAction(active_motion_)));

  this->set_last_trajectory(traj);
  return traj;
}

dynamic::Trajectory BehaviorMotionPrimitives::PlanFrenet(
    float delta_time,
    const world::ObservedWorld& observed_world) {
  using modules::world::map::FrenetState;
  using modules::world::map::FrenetStateToDynamicState;
  using modules::geometry::Point2d;
  
  const geometry::Line& center_line = observed_world.get_local_map()->get_driving_corridor().get_center();
  BARK_EXPECT_TRUE(!center_line.obj_.empty());

  // define running states
  const dynamic::State ego_vehicle_state = observed_world.current_ego_state();
  const FrenetState current_frenet_state(ego_vehicle_state, center_line);
  float current_long_vel = current_frenet_state.vlon;
  float current_lat_dist = current_frenet_state.lat;

  // define input dynamic variables
  auto frenet_input = motion_primitives_[active_motion_];
  const float longitudinal_acceleration = frenet_input[0];
  const float lat_direction = frenet_input[1];
  float lateral_velocity = 0.0f; 
  if(lat_direction == 0.0f) {
    lateral_velocity=0.0f;
  } else if (lat_direction < 0.0f) {
    lateral_velocity = + std::max(0.17*current_long_vel, 0.5); // From konsti paper
  } else {
    lateral_velocity = - std::max(0.17*current_long_vel, 0.5); // From konsti paper
  }

  // integration
  float s_start  = current_frenet_state.lon;
  float start_time = observed_world.get_world_time();
  float sline = s_start;
  float run_time = start_time;

  const float dt = integration_time_delta_; 
  const int num_trajectory_points = static_cast<int>(std::ceil(delta_time / dt))+1;
  dynamic::Trajectory traj(num_trajectory_points, int(StateDefinition::MIN_STATE_SIZE));
  for (int i = 0; i < traj.rows(); i++) {
    const dynamic::State current_state = FrenetStateToDynamicState(
      FrenetState(sline, current_lat_dist, current_long_vel, lateral_velocity), center_line);
    traj(i, StateDefinition::TIME_POSITION) = run_time; 
    traj(i, StateDefinition::X_POSITION) = current_state[dynamic::StateDefinition::X_POSITION];
    traj(i, StateDefinition::Y_POSITION) = current_state[dynamic::StateDefinition::Y_POSITION];
    traj(i, StateDefinition::THETA_POSITION) = current_state[dynamic::StateDefinition::THETA_POSITION];
    traj(i, StateDefinition::VEL_POSITION) = current_state[dynamic::StateDefinition::VEL_POSITION];
    current_lat_dist = current_lat_dist + lateral_velocity * dt;
    current_long_vel = current_long_vel + longitudinal_acceleration * dt;
    sline = sline + current_long_vel * dt;
    run_time += dt;
  }

  set_last_action(Action(DiscreteAction(active_motion_)));
  this->set_last_trajectory(traj);
  return traj;
}

BehaviorMotionPrimitives::MotionIdx BehaviorMotionPrimitives::AddMotionPrimitive(const Input& dynamic_input) {  
  motion_primitives_.push_back(dynamic_input);
  return motion_primitives_.size()-1;
}

void BehaviorMotionPrimitives::ActionToBehavior(const MotionIdx& motion_idx) {
  active_motion_ = motion_idx;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
