#include "turtlelib/diff_drive.hpp"

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/test_utils.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using Catch::Matchers::WithinRel;

namespace turtlelib {

TEST_CASE("WheelConfig") {
  DiffDrive::WheelConfig config1{1.0, 2.4};
  DiffDrive::WheelConfig delta_config{0.1, -0.4};
  config1 += delta_config;
  REQUIRE_THAT(config1.left, WithinRel(1.1));
  REQUIRE_THAT(config1.right, WithinRel(2.0));
}

TEST_CASE("FK") {
  // At init, robot is facing Y dir
  //        x
  //        │
  //     y──┘b
  //   y    (1,1)
  //   │
  //   └──x
  // s
  double track_body_width = 0.1; // half of wheel-wheel distance.
  double wheel_r = 0.1;
  double init_angle = PI/2;
  Transform2D init_pose = {{1.0, 1.0}, init_angle};
  DiffDrive ddrive(track_body_width, wheel_r, init_pose);

  SECTION("Move Streight") {
    // distance traveled is theta*r
    DiffDrive::WheelConfig straight10m_wheel_delta = {10.0 / wheel_r,
                                                      10.0 / wheel_r};
    INFO("straight10m_wheel_delta left " << straight10m_wheel_delta.left
                                         << "  right "
                                         << straight10m_wheel_delta.right);

    Twist2D body_tw_10m = ddrive.TwistFromDeltaWheel(straight10m_wheel_delta);
    REQUIRE_THAT(body_tw_10m.omega, WithinRel(0.0));
    REQUIRE_THAT(body_tw_10m.x, WithinRel(10.0));
    REQUIRE_THAT(body_tw_10m.y, WithinRel(0.0));

    ddrive.UpdateBodyConfig(straight10m_wheel_delta);

    REQUIRE_THAT(ddrive.GetBodyConfig(),
                 Transform2DWithinRel(Transform2D{{1.0, 11.0}, init_angle}));
  }

  SECTION("Rotate in place") {
    auto angle_turned = GENERATE(
        Catch::Generators::take(5, Catch::Generators::random(-PI + 1e-4, PI)));

    // Generate wheel travel from turning.
    // arc = theta*r
    // right wheel increment with positive turn
    // left wheel decrease with positive turn.
    double left_linear_dis = -angle_turned * (track_body_width);
    double right_linear_dis = angle_turned * (track_body_width);

    DiffDrive::WheelConfig rotate_wheel_delta = {left_linear_dis / wheel_r,
                                                 right_linear_dis / wheel_r};
    ddrive.UpdateBodyConfig(rotate_wheel_delta);
    REQUIRE_THAT(ddrive.GetBodyConfig(),
                 Transform2DWithinRel(
                     Transform2D{{1.0, 1.0}, init_angle + angle_turned}));
  }
}
} // namespace turtlelib