// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_MAP_FRENET_HPP_
#define MODULES_WORLD_MAP_FRENET_HPP_

#include "modules/geometry/line.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"


namespace modules {
namespace world {
namespace map {

struct FrenetPosition {

FrenetPosition() : lon(0.0f), lat(0.0f) {}
FrenetPosition(const double& longitudinal, const double& lateral) : lon(longitudinal), lat(lateral) {}
FrenetPosition(const modules::geometry::Point2d& position, const modules::geometry::Line& path);

double lon;
double lat;

};

struct FrenetState : public FrenetPosition {

FrenetState() : FrenetPosition() {}
FrenetState(const double& longitudinal, const double& lateral,
            const double& vlongitudinal, const double& vlateral) :
                 FrenetPosition(longitudinal, lateral),
                 vlon(vlongitudinal),
                 vlat(vlateral) {}
FrenetState(const modules::models::dynamic::State& state, const modules::geometry::Line& path);

double vlon;
double vlat;

};

modules::models::dynamic::State FrenetStateToDynamicState(
      const FrenetState& frenet_state,  const modules::geometry::Line& path);

}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_FRENET_HPP_