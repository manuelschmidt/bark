// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/map/frenet.hpp"

namespace modules {
namespace world {
namespace map {

using modules::geometry::Point2d;
using modules::geometry::Line;
using modules::geometry::operator-;

FrenetPosition::FrenetPosition(const Point2d& position, const Line& path) {
  namespace bg = boost::geometry;

  // First extract nearest point, extract longitudinal coordinate
  std::tuple<Point2d, double, uint> nearest = modules::geometry::get_nearest_point_and_s(path, position); 
  lon = std::get<1>(nearest);

  // calculate lateral coordinate value manually to avoid researching the nearest point
  auto nearest_point = std::get<0>(nearest);
  auto x_diff = bg::get<0>(nearest_point) - bg::get<0>(position);
  auto y_diff = bg::get<1>(nearest_point) - bg::get<1>(position);
  double lat_val = sqrt(x_diff*x_diff + y_diff*y_diff);

  // calculate sign of lateral coordinate
  auto tangent_angle = modules::geometry::get_tangent_angle_at_s(path, lon);
  auto direction_vector = position - nearest_point;
  double diff = modules::geometry::signed_angle_diff(tangent_angle , atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? -1 : ((diff < 0) ? 1 : 0);

  lat = lat_val*sign;
}

FrenetState::FrenetState(const modules::models::dynamic::State& state, const modules::geometry::Line& path) {
  namespace bg = boost::geometry;
  using st = modules::models::dynamic::StateDefinition;

  // First extract nearest point, extract longitudinal coordinate
  modules::geometry::Point2d pos(state(st::X_POSITION), state(st::Y_POSITION));
  std::tuple<Point2d, double, uint> nearest = modules::geometry::get_nearest_point_and_s(path, pos); 
  lon = std::get<1>(nearest);

  // calculate lateral coordinate value manually to avoid researching the nearest point
  auto nearest_point = std::get<0>(nearest);
  auto x_diff = bg::get<0>(nearest_point) - bg::get<0>(pos);
  auto y_diff = bg::get<1>(nearest_point) - bg::get<1>(pos);
  double lat_val = sqrt(x_diff*x_diff + y_diff*y_diff);

  // calculate sign of lateral coordinate
  auto tangent_angle = modules::geometry::norm_0_2PI(modules::geometry::get_tangent_angle_at_s(path, lon));
  auto direction_vector = pos - nearest_point;
  double diff = modules::geometry::signed_angle_diff(tangent_angle , atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? -1 : ((diff < 0) ? 1 : 0);
  lat = lat_val*sign;

  // velocities
  const auto velocity = state[modules::models::dynamic::StateDefinition::VEL_POSITION];
  const auto orientation = modules::geometry::norm_0_2PI(state[modules::models::dynamic::StateDefinition::THETA_POSITION]);
  vlon = cos(std::abs(orientation-tangent_angle)) * velocity;
  vlat = sqrt(velocity*velocity - vlon*vlon)*sign;  
}


modules::models::dynamic::State FrenetStateToDynamicState(
      const FrenetState& frenet_state,  const modules::geometry::Line& path) {
  namespace bg = boost::geometry;
  using st = modules::models::dynamic::StateDefinition;
  using namespace modules::geometry;
  // calculate position
  const auto line_point = get_point_at_s(path, frenet_state.lon);
  const auto line_angle = get_tangent_angle_at_s(path, frenet_state.lon);
  const auto normal = get_normal_at_s(path, frenet_state.lon);
  const auto position = line_point + normal * frenet_state.lat; // todo check if direction fits sign definition

  // calculate angle from frenet velocities
  const auto angle = atan2(frenet_state.vlat, frenet_state.vlon) + line_angle; // todo che

  const auto velocity = sqrt(frenet_state.vlon * frenet_state.vlon + 
                            frenet_state.vlat * frenet_state.vlat);
  
  // build state
  modules::models::dynamic::State state(static_cast<int>(st::MIN_STATE_SIZE));
  state(st::TIME_POSITION) = 0.0f;
  state(st::X_POSITION) = bg::get<0>(position);
  state(st::Y_POSITION) = bg::get<1>(position);
  state(st::THETA_POSITION) = angle;
  state(st::VEL_POSITION) = velocity;
  return state;
  }

}  // namespace map
}  // namespace world
}  // namespace modules