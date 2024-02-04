#include "turtlelib/diff_drive.hpp"

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/test_utils.hpp"

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstddef>

using Catch::Matchers::WithinRel;

namespace turtlelib {

TEST_CASE("WheelConfig") {
  WheelConfig config1{1.0, 2.4};
  WheelConfig delta_config{0.1, -0.4};
  config1 += delta_config;
  REQUIRE_THAT(config1, WheelConfigWithinRel({1.1, 2.0}));
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
  double init_angle = PI / 2;
  Transform2D init_pose = {{1.0, 1.0}, init_angle};
  WheelConfig init_wheel_config{0.1, 0.2};
  DiffDrive ddrive(track_body_width, wheel_r, init_pose, init_wheel_config);

  SECTION("Move Straight") {
    // distance traveled is theta*r
    WheelVelocity straight10m_wheel_delta = {10.0 / wheel_r, 10.0 / wheel_r};

    CAPTURE(straight10m_wheel_delta);

    // We quickly check the Twist calculation function is working correctly
    Twist2D body_tw_10m = ddrive.TwistFromWheelDelta(straight10m_wheel_delta);
    SECTION("FK") {
      REQUIRE_THAT(body_tw_10m.omega, WithinRel(0.0));
      REQUIRE_THAT(body_tw_10m.x, WithinRel(10.0));
      REQUIRE_THAT(body_tw_10m.y, WithinRel(0.0));

      SECTION("Update with velocity") {

        INFO("Current body config " << ddrive.GetBodyConfig());
        ddrive.UpdateBodyConfigWithVel(straight10m_wheel_delta);

        REQUIRE_THAT(ddrive.GetBodyConfig(), Transform2DWithinRel(Transform2D{
                                                 {1.0, 11.0}, init_angle}));
      }
      SECTION("Update with new value") {
        INFO("Current body config " << ddrive.GetBodyConfig());
        WheelConfig new_config =
            init_wheel_config + WheelConfig{0.01 / wheel_r, 0.01 / wheel_r};
        ddrive.UpdateRobotConfig(new_config);
        REQUIRE_THAT(ddrive.GetBodyConfig(), Transform2DWithinRel(Transform2D{
                                                 {1.0, 1.01}, init_angle}));
      }
    }
    SECTION("IK test") {
      CAPTURE(body_tw_10m);
      REQUIRE_THAT(ddrive.CommandFromTwist(body_tw_10m),
                   WheelConfigWithinRel(straight10m_wheel_delta));
    }
  }

  SECTION("Rotate in place") {
    auto angle_turned = GENERATE(
        Catch::Generators::take(5, Catch::Generators::random(-PI + 1e-4, PI)));

    // Generate wheel travel from turning.
    // arc = theta*r
    // right wheel increment with positive turn
    // left wheel decrease with positive turn.

    INFO("Angle turned " << angle_turned << " deg " << rad2deg(angle_turned));
    double left_linear_dis = -angle_turned * (track_body_width);
    double right_linear_dis = angle_turned * (track_body_width);

    WheelVelocity rotate_wheel_delta = {left_linear_dis / wheel_r,
                                        right_linear_dis / wheel_r};

    SECTION("FK test") {
      CAPTURE(rotate_wheel_delta);
      ddrive.UpdateRobotConfig(rotate_wheel_delta + init_wheel_config);
      REQUIRE_THAT(ddrive.GetBodyConfig(),
                   Transform2DWithinRel(
                       Transform2D{{1.0, 1.0}, init_angle + angle_turned}));
    }

    SECTION("IK test") {

      INFO(ddrive.TwistFromWheelDelta(rotate_wheel_delta));
      REQUIRE_THAT(ddrive.CommandFromTwist(Twist2D{angle_turned, 0.0, 0.0}),
                   WheelConfigWithinRel(rotate_wheel_delta));
    }
  }

  SECTION("Drive to left") {
    // We drive a quarter circle with rotation center at {s}(-3.1).
    // This generate a 90deg arc with turning radius of 4.
    // This big move will certainly have issue with wheel encoder overflow.
    // So the move is broken up to many small segments.

    // clang-format off
    
    //  x──┐b'
    //     │ (-3,5)
    //     y
    //
    //
    //          Turning
    //          Rad = 4     x
    //                      │
    //     ┼             y──┘b
    //  (-3,1)         y    (1,1)
    //  Turning        │
    //  Center        s└──x

    // clang-format on

    // The screw axis for this rotation is {1 , 4, 0}
    double turning_rad = 4.0;
    // Twist is screw * omega
    Twist2D expected_overall_twist = {1.0 * (PI / 2), turning_rad * (PI / 2),
                                      0.0};
    // Left wheel turning rad 4.0 - track_body_width
    // Right wheel turning rad 4.0 + track_body_width
    WheelConfig wheel_delta{
        (PI / 2) * (turning_rad - track_body_width) / wheel_r,
        (PI / 2) * (turning_rad + track_body_width) / wheel_r};
    Transform2D expected_end_tf{{-3.0, 5.0}, PI};

    SECTION("FK test") {
      // We know the right wheel will have larger turning radius. We'll just
      // scale base on that
      size_t iteration = 25;
      REQUIRE(wheel_delta.right / double(iteration) < PI);
      WheelVelocity step_wheel_increment{wheel_delta.left / double(iteration),
                                         wheel_delta.right / double(iteration)};
      CAPTURE(step_wheel_increment);
      auto current_wheel_config = init_wheel_config;
      for (size_t i = 0; i < iteration; ++i) {
        current_wheel_config += step_wheel_increment;
        ddrive.UpdateRobotConfig(current_wheel_config);
        // Logging are scopped. So seems like log in loop will not exit the loop
        UNSCOPED_INFO("After iteration: "
                      << i << "\nCurrent wheel " << current_wheel_config
                      << "\nCurrent Body " << ddrive.GetBodyConfig());
      }
      // After all of this, then we check if robot is there.
      REQUIRE_THAT(ddrive.GetBodyConfig(),
                   Transform2DWithinRel(expected_end_tf));
    }
    SECTION("IK test") {

      REQUIRE_THAT(ddrive.CommandFromTwist(expected_overall_twist),
                   WheelConfigWithinRel(wheel_delta));
    }
  }
}
} // namespace turtlelib