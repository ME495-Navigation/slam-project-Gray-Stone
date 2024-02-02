#include "turtlelib/se2d.hpp"

#include "turtlelib/test_utils.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_templated.hpp>
#include <sstream>

using Catch::Matchers::Equals;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace turtlelib {

TEST_CASE("Twist 2d stream test", "[Twist2D]") {
  // Setup needed later
  Twist2D twist_a{10.5, 2, 1};
  // Point2D point_b{3, 1};
  std::stringstream ss;
  Twist2D out_twist;

  SECTION("OutStream") {

    ss << twist_a;
    REQUIRE_THAT(ss.str(), Equals("[10.5 2 1]"));
    twist_a.x = 3.33;
    ss.str(std::string());
    ss << twist_a;
    REQUIRE_THAT(ss.str(), Equals("[10.5 3.33 1]"));
    twist_a.y = 10.678;
    ss.str(std::string());
    ss << twist_a;
    REQUIRE_THAT(ss.str(), Equals("[10.5 3.33 10.678]"));
    twist_a.omega = -10.3;
    ss.str(std::string());
    ss << twist_a;
    REQUIRE_THAT(ss.str(), Equals("[-10.3 3.33 10.678]"));
  }
  SECTION("InStream") {
    ss << twist_a;

    ss >> out_twist;
    REQUIRE_THAT(out_twist.omega, WithinRel(twist_a.omega));
    REQUIRE_THAT(out_twist.x, WithinRel(twist_a.x));
    REQUIRE_THAT(out_twist.y, WithinRel(twist_a.y));
    SECTION("InStream2") {
      ss.str("1203.9 -10   20.12345");
      ss >> out_twist;
      REQUIRE_THAT(out_twist.omega, WithinRel(1203.9));
      REQUIRE_THAT(out_twist.x, WithinRel(-10.0f));
      REQUIRE_THAT(out_twist.y, WithinRel(20.12345));
    }
    SECTION("Instream3") {
      ss.str("1 1 1");
      ss >> out_twist;
      REQUIRE_THAT(out_twist.omega, WithinRel(1.0));
      REQUIRE_THAT(out_twist.x, WithinRel(1.0));
      REQUIRE_THAT(out_twist.y, WithinRel(1.0));
    }
  }
}

TEST_CASE("Transform2D constructor and getter test", "[Transform2D]") {
  SECTION("base_constructor") {
    Transform2D tf;
    REQUIRE_THAT(tf.translation().x, WithinRel(0.0));
    REQUIRE_THAT(tf.translation().y, WithinRel(0.0));
    REQUIRE_THAT(tf.rotation(), WithinRel(0.0));
  }

  // These generators is likely to have namespace collision with cmath
  auto angle = GENERATE(
      Catch::Generators::take(5, Catch::Generators::random(-PI + 1e-4, PI)));
  auto x = GENERATE(
      Catch::Generators::take(5, Catch::Generators::random(-1e10, 1e10)));
  auto y = GENERATE(
      Catch::Generators::take(5, Catch::Generators::random(-1e10, 1e10)));
  SECTION("angle_constructor") {
    Transform2D angle_tf{angle};
    REQUIRE_THAT(angle_tf.rotation(), WithinRel(angle));
  }
  SECTION("translation_constructor") {
    Transform2D trans_tf{Vector2D{x, y}};
    REQUIRE_THAT(trans_tf.translation().x, WithinRel(x));
    REQUIRE_THAT(trans_tf.translation().y, WithinRel(y));
  }
  SECTION("Combo_constructor") {
    Transform2D tf{Vector2D{x, y}, angle};
    REQUIRE_THAT(tf.translation().x, WithinRel(x));
    REQUIRE_THAT(tf.translation().y, WithinRel(y));
    REQUIRE_THAT(tf.rotation(), WithinRel(angle));
  }
}

TEST_CASE("Transform 2d stream test", "[Twist2D]") {
  // Setup needed later
  Transform2D tf{{10.5, 2}, deg2rad(18)};
  // Point2D point_b{3, 1};

  std::stringstream ss;
  SECTION("OutStream") {

    std::stringstream ss;
    ss << tf;
    REQUIRE_THAT(ss.str(), Equals("deg: 18 x: 10.5 y: 2"));
  }
  SECTION("InStream format1") {
    Transform2D tf2;

    ss << tf;
    ss >> tf2;

    REQUIRE_THAT(tf2.translation().x, WithinRel(10.5));
    REQUIRE_THAT(tf2.translation().y, WithinRel(2.0));
    REQUIRE_THAT(tf2.rotation(), WithinRel(deg2rad(18.0)));
  }
  SECTION("InStream format2") {
    ss.str("11.2    22.5 80.8");
    ss >> tf;
    INFO(tf.translation().x);
    REQUIRE_THAT(tf.rotation(), WithinRel(deg2rad(11.2)));
    REQUIRE_THAT(tf.translation().x, WithinRel(22.5));
    REQUIRE_THAT(tf.translation().y, WithinRel(80.8));
  }
}

TEST_CASE("Twist integrate", "[Twist]") {

  // Init pose
  SECTION("Translate only") {
    REQUIRE_THAT(integrate_twist(Twist2D{0, 10.1, 2.2}),
                 Transform2DWithinRel({{10.1, 2.2}, 0.0}));
  }

  SECTION("Y twist") {

    //         b'x
    //       ▲
    // b'y   │
    //   ◄───┘      Screw( xy=(0,2),rot=90)
    //     (-2,2)

    //                    by
    //                   ▲
    //       │           │   bx
    //       └──         └──►
    //     (-2,0)        (0,0)

    // For screw, the thing is normalized with omega=1. So to generate a twist
    // Needs to multiply by rotation amount.
    Twist2D twist{1.0 * PI / 2, 0.0, 2.0 * PI / 2};
    REQUIRE_THAT(integrate_twist(twist),
                 Transform2DWithinRel({{-2.0, 2.0}, PI / 2}));
  }
  // clang-off
  //     b'x
  //       ▲
  // b'y   │
  //   ◄───┘      Twist( (0,2),90)
  //     (-1,2)

  //                    by
  //             sy    ▲
  //       │     ▲  sx │   bx
  //       └──   └─►   └──►
  //     (-1,0)        (1,0)
}

// The following are imported tests from Srikanth.
TEST_CASE("operator()(Vector2D v)", "[transform]") { // Srikanth, Schelbert
  double test_rot = -PI / 2.0;
  Vector2D test_vec = {1.0, 1.0};
  Transform2D T_ab{{test_vec}, test_rot};
  Vector2D v_b{1, 1};
  Vector2D v_a = T_ab(v_b);
  REQUIRE_THAT(v_a.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
  REQUIRE_THAT(v_a.y, Catch::Matchers::WithinAbs(-1.0, 1e-5));
}

TEST_CASE("operator()(Point2D v)", "[transform]") { // Srikanth, Schelbert
  double test_rot = -PI / 2.0;
  Vector2D test_vec = {1.0, 1.0};
  Transform2D T_ab{{test_vec}, test_rot};
  Point2D p_b{1, 1};
  Point2D p_a = T_ab(p_b);
  REQUIRE_THAT(p_a.x, Catch::Matchers::WithinAbs(2.0, 1e-5));
  REQUIRE_THAT(p_a.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("operator()(Twist2D t)", "[transform]") { // Srikanth Schelbert
  double test_rot = PI / 2.0;
  Vector2D test_vec = {0.0, 1.0};
  Transform2D T_ab{{test_vec}, test_rot};
  Twist2D T_b{1, 1, 1};
  Twist2D T_a = T_ab(T_b);
  REQUIRE_THAT(T_a.omega, Catch::Matchers::WithinAbs(1.0, 1e-5));
  REQUIRE_THAT(T_a.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(T_a.y, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("inverse - inv()", "[transform]") { // Srikanth, Schelbert
  double test_rot = -PI / 2.0;
  double test_x = 1.0;
  double test_y = 1.0; // Just make this a vector
  Transform2D T_test{{test_x, test_y}, test_rot};
  Transform2D T_test_inv = T_test.inv();
  REQUIRE(T_test.inv().rotation() == -test_rot);
  REQUIRE_THAT(T_test_inv.translation().x,
               Catch::Matchers::WithinAbs(1.0, 1e-5));
  REQUIRE_THAT(T_test_inv.translation().y,
               Catch::Matchers::WithinAbs(-1.0, 1e-5));
}

TEST_CASE("matrix mult operator - operator*=",
          "[transform]") { // Srikanth, Schelbert
  Vector2D test_trans_ab = {1.0, 2.0};
  double test_rot_ab = 0.0;
  Vector2D test_trans_bc = {2.0, 3.0};
  double test_rot_bc = PI / 2;
  Transform2D T_ab1 = {test_trans_ab, test_rot_ab};
  Transform2D T_ab2 = {test_trans_ab, test_rot_ab};
  Transform2D T_ab3 = {test_trans_ab,
                       test_rot_ab}; // made 3 since fcn returns overwritten tf
  Transform2D T_bc = {test_trans_bc, test_rot_bc};
  REQUIRE_THAT((T_ab1 *= T_bc).translation().x,
               Catch::Matchers::WithinAbs(3.0, 1e-5));
  REQUIRE_THAT((T_ab2 *= T_bc).translation().y,
               Catch::Matchers::WithinAbs(5.0, 1e-5));
  REQUIRE_THAT((T_ab3 *= T_bc).rotation(),
               Catch::Matchers::WithinAbs(PI / 2.0, 1e-5));
}

} // namespace turtlelib