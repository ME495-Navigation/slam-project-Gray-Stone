#include "turtlelib/se2d.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <sstream>

using Catch::Matchers::Equals;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace turtlelib {

TEST_CASE("Twist 2d ", "[Twist2D]") {
  // Setup needed later
  Twist2D twist_a{10.5, 2, 1};
  // Point2D point_b{3, 1};

  SECTION("OutStream") {

    std::stringstream ss;
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
    std::stringstream ss;
    ss << twist_a;

    Twist2D out_twist;
    ss >> out_twist;
    REQUIRE_THAT(out_twist.x, WithinRel(twist_a.x));
    REQUIRE_THAT(out_twist.y, WithinRel(twist_a.y));
    REQUIRE_THAT(out_twist.omega, WithinRel(twist_a.omega));

    ss.str("1203.9 -10   20.12345");
    ss >> out_twist;
    REQUIRE_THAT(out_twist.x, WithinRel(-10.0f));
    REQUIRE_THAT(out_twist.y, WithinRel(20.12345));
    REQUIRE_THAT(out_twist.omega, WithinRel(1203.9));
  }
}

TEST_CASE("Transform2D constructor test", "[Transform2D]") {
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

TEST_CASE("transform math", "[Transform2D]") {}
} // namespace turtlelib