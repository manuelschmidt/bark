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
                                             "the size of the time steps used within the euler integration loop", 0.02)),
    use_frenet_motions(params->get_bool("BehaviorMotionPrimitives::use_frenet_motions",
                                             "If true motion primitives inputs are interpreted as longitudinal acceleration 
                                                and lateral velocity.", false))
    {}

dynamic::Trajectory BehaviorMotionPrimitives::Plan(float delta_time,
    const world::ObservedWorld& observed_world) {
      if(use_frenet_motions) {
        return PlanContinuous(delta_time, observed_world);
      } else {
        return PlanFrenetPosition(delta_time, observed_world);
      }
    }

dynamic::Trajectory BehaviorMotionPrimitives::PlanContinuous(
    float delta_time,
    const world::ObservedWorld& observed_world) {
  dynamic::State ego_vehicle_state = observed_world.current_ego_state();
  double start_time = observed_world.get_world_time();
   const float dt = integration_time_delta_; 
  const int num_trajectory_points = static_cast<int>(std::ceil(delta_time / dt))+1;

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

dynamic::Trajectory BehaviorMotionPrimitives::PlanFrenetPosition(
    float delta_time,
    const world::ObservedWorld& observed_world) {
  using modules::world::map::FrenetPosition;
  using modules::geometry::Point2d;
  
  const dynamic::State ego_vehicle_state = observed_world.current_ego_state();
  double start_time = observed_world.get_world_time();
   const float dt = integration_time_delta_; 
  const int num_trajectory_points = static_cast<int>(std::ceil(delta_time / dt))+1;

  geometry::Line center_line = observed_world.get_local_map()->get_driving_corridor().get_center();
  const FrenetState current_frenet_state(ego_vehicle_state, center_line);

  dynamic::Trajectory traj(num_trajectory_points, int(dynamic::StateDefinition::MIN_STATE_SIZE));
  auto frenet_input = motion_primitives_[active_motion_];

  // check whether linestring is empty
  if (line.obj_.size()>0) {
    float s_start  = current_frenet_coord.lon;
    float start_time = observed_world.get_world_time();
    const float current_vel = ego_vehicle_state(StateDefinition::VEL_POSITION);
    float current_long_vel = start_frenet_state.vlon;
    float current_lat_dist = start_frenet_coord.lat;
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
    BARK_EXPECT_TRUE(!std::isnan(acceleration));
    float sline = s_start;
    // v = s/t
    float run_time = start_time;
    for (int i = 0; i < traj.rows(); i++) {
      sline = sline + current_long_vel * sample_time;
      geometry::Point2d line_point = get_point_at_s(line, sline); 
      geometry::Point2d normal = get_normal_at_s(line, sline);
      float traj_angle = 
      traj(i, StateDefinition::TIME_POSITION) = run_time; 
      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<0>(traj_point)));
      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<1>(traj_point)));
      traj(i, StateDefinition::X_POSITION) = boost::geometry::get<0>(traj_point);
      traj(i, StateDefinition::Y_POSITION) = boost::geometry::get<1>(traj_point);
      traj(i, StateDefinition::THETA_POSITION) = traj_angle;
      traj(i, StateDefinition::VEL_POSITION) = current_vel;
      current_lat_dist = current_lat_dist + lateral_velocity * sample_time;
      current_long_vel = current_long_vel + longitudinal_acceleration * sample_time;
      run_time += sample_time;
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
