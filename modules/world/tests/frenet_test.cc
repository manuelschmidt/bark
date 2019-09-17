// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/frenet.hpp"

using namespace modules::world::map;
using namespace modules::geometry;
using namespace modules::models::dynamic;
using st = modules::models::dynamic::StateDefinition;

void test_state(const float x, const float y,
                const float theta, const float v,
                const Line& line) {
  State state(static_cast<int>(st::MIN_STATE_SIZE));
  state << 0.0f, x, y, theta, v;

  FrenetState frenet_state(state, line);
  auto state_conv = FrenetStateToDynamicState(frenet_state, line);

  EXPECT_NEAR(state(st::X_POSITION),state_conv(st::X_POSITION),0.001);
  EXPECT_NEAR(state(st::Y_POSITION),state_conv(st::Y_POSITION),0.001);
  EXPECT_NEAR(state(st::VEL_POSITION),state_conv(st::VEL_POSITION),0.001);
  EXPECT_NEAR(state(st::THETA_POSITION),state_conv(st::THETA_POSITION),0.001);
}

TEST(frenet_state, straight_line_right) {

  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.add_point(Point2d(1,0));
  line.add_point(Point2d(2,0));
  line.add_point(Point2d(10,0));

  // state on path with orientation on path
  test_state(3, 0, 0, 5, line);

  // state on left side of path with orientation on path
  test_state(2.5, 1, 0, 5, line);

   // state on right side of path with orientation on path
  test_state(8.2, -1, 0, 5, line);
}

TEST(frenet_state, straight_line_top) {

  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.add_point(Point2d(0,1));
  line.add_point(Point2d(0,2));
  line.add_point(Point2d(0,10));

  std::cout <<"test";

  // state on path with orientation on path
  test_state(0 , 3, B_PI_2, 5, line);

  // state on left side of path with orientation on path
  test_state(1, 4, B_PI_2, 5, line);

   // state on right side of path with orientation on path
  test_state(-1, 5, B_PI_2, 5, line);
}