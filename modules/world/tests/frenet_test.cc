// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/frenet.hpp"


TEST(frenet_state, straigt_line) {
  using namespace modules::world::map;
  using namespace modules::geometry;
  using namespace modules::models::dynamic;

  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.add_point(Point2d(1,0));
  line.add_point(Point2d(2,0));
  line.add_point(Point2d(10,0));


}