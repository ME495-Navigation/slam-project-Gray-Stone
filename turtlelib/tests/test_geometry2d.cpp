#include "turtlelib/geometry2d.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <sstream>
namespace turtlelib {

using Catch::Matchers::Equals;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
TEST_CASE("Normalizing angles") {
  // double base_angle = 3* PI /2;
  REQUIRE_THAT(normalize_angle(PI), WithinRel(PI));
  REQUIRE_THAT(normalize_angle(-PI), WithinRel(PI));
  REQUIRE_THAT(normalize_angle(0), WithinRel(0.0));
  REQUIRE_THAT(normalize_angle(-PI / 4), WithinRel(-PI / 4));
  REQUIRE_THAT(normalize_angle(3 * PI / 2), WithinRel(-PI / 2));
  REQUIRE_THAT(normalize_angle(-5 * PI / 2), WithinRel(-PI / 2));
  double rad_step_size = 1e-6;
  for (int circle_count = -4; circle_count < 4; ++circle_count) {
    // Sweep all angles within -PI to PI
    for (double base_angle = -PI + rad_step_size; base_angle < PI;
         base_angle += rad_step_size) {
      REQUIRE_THAT(normalize_angle(base_angle + circle_count * 2 * PI),
                   WithinAbs(base_angle, 1e-6));
    }
  }
};

TEST_CASE("Point2D tests", "[point2d]") {
  // Setup needed later
  Point2D point_a{2, 1};
  // Point2D point_b{3, 1};

  SECTION("OutStream") {

    std::stringstream ss;
    ss << point_a;
    REQUIRE_THAT(ss.str(), Equals("[2 1]"));
    point_a.x = 3.33;
    ss.str(std::string());
    ss << point_a;
    REQUIRE_THAT(ss.str(), Equals("[3.33 1]"));
    point_a.y = 10.678;
    ss.str(std::string());
    ss << point_a;
    REQUIRE_THAT(ss.str(), Equals("[3.33 10.678]"));
  }
  SECTION("InStream") {
    std::stringstream ss;
    ss << point_a;

    Point2D out_point;
    ss >> out_point;
    REQUIRE_THAT(out_point.x, WithinRel(point_a.x));
    REQUIRE_THAT(out_point.y, WithinRel(point_a.y));

    ss.str("10   20.12345");
    ss >> out_point;
    REQUIRE_THAT(out_point.x, WithinRel(10.0f));
    REQUIRE_THAT(out_point.y, WithinRel(20.12345));
  }
}

TEST_CASE("Vector2D stream tests", "[Vector2D]") {
  // Setup needed later
  Vector2D vector_a{2, 1};
  // Vector2D point_b{3, 1};

  SECTION("OutStream") {
    std::stringstream ss;
    ss << vector_a;
    REQUIRE_THAT(ss.str(), Equals("[2 1]"));
    vector_a.x = 3.33;
    ss.str(std::string());
    ss << vector_a;
    REQUIRE_THAT(ss.str(), Equals("[3.33 1]"));
    vector_a.y = 10.678;
    ss.str(std::string());
    ss << vector_a;
    REQUIRE_THAT(ss.str(), Equals("[3.33 10.678]"));
  }
  SECTION("InStream") {
    std::stringstream ss;
    ss << vector_a;

    Vector2D out_vec;
    ss >> out_vec;
    REQUIRE_THAT(out_vec.x, WithinRel(vector_a.x));
    REQUIRE_THAT(out_vec.y, WithinRel(vector_a.y));

    ss.str("10   20.12345");
    ss >> out_vec;
    REQUIRE_THAT(out_vec.x, WithinRel(10.0f));
    REQUIRE_THAT(out_vec.y, WithinRel(20.12345));
  }
}

TEST_CASE("Vector2D Normalize", "[Vector2D]") {

  auto scale_factor = GENERATE(
      Catch::Generators::take(5, Catch::Generators::random(-1e20, 1e20)));

  Vector2D vector_a{0.5547 * scale_factor, 0.8320 * scale_factor};
  auto normed_vector = vector_a.normalize();

  // Need to also be careful not to include 0s
  if (scale_factor < 0) {

    REQUIRE_THAT(normed_vector.x, WithinAbs(-0.5547, 1e-3));
    REQUIRE_THAT(normed_vector.y, WithinAbs(-0.8320, 1e-3));
  } else if (scale_factor > 0) {

    REQUIRE_THAT(normed_vector.x, WithinAbs(0.5547, 1e-3));
    REQUIRE_THAT(normed_vector.y, WithinAbs(0.8320, 1e-3));
  }
}

TEST_CASE("Point - Point") {
  Point2D point_tail{10, 10};
  Point2D point_head{15, 11};
  Vector2D vec = point_head - point_tail;
  REQUIRE_THAT(vec.x, WithinRel(5.0));
  REQUIRE_THAT(vec.y, WithinRel(1.0));
}

TEST_CASE("Point + Vector") {
  Point2D base_point{1, 3};
  Vector2D disp{10, 4};
  Point2D result = base_point + disp;
  REQUIRE_THAT(result.x, WithinRel(11.0));
  REQUIRE_THAT(result.y, WithinRel(7.0));
}
} // namespace turtlelib