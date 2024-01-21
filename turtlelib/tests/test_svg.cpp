#include "turtlelib/svg.hpp"

#include <catch2/catch_test_macros.hpp>

#include <catch2/matchers/catch_matchers_string.hpp>

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::EndsWith;
using Catch::Matchers::StartsWith;

namespace turtlelib {

TEST_CASE("svg test ", "[svg]") {
  Svg svg;

  Transform2D space_frame{{50, 50}, deg2rad(22.2)};

  INFO("Generated point: ");

  SECTION("Point Object test", "[svg]") {

    svg.AddObject(Point2D{10, -10}, space_frame);
    std::string generated_str = svg.GetConstructedObjects();

    REQUIRE_THAT(
        generated_str,
        StartsWith("\n<g transform=\"translate(50,50) rotate(22.2)\" >"));
    REQUIRE_THAT(generated_str, EndsWith("</g>"));
    REQUIRE_THAT(generated_str,
                 ContainsSubstring(
                     "<circle cx=\"10\" cy=\"-10\" r=\"3\" stroke=\"blue\" "
                     "fill=\"tan\" stroke-width=\"1\" />"));
  }

  SECTION("Vector2D Object test", "[svg]") {

    svg.AddObject(Vector2D{-5, 9}, space_frame);
    std::string generated_str = svg.GetConstructedObjects();

    REQUIRE_THAT(
        generated_str,
        StartsWith("\n<g transform=\"translate(50,50) rotate(22.2)\" >"));
    REQUIRE_THAT(generated_str, EndsWith("</g>"));

    REQUIRE_THAT(generated_str,
                 ContainsSubstring("<line x1=\"-5\" y1=\"9\" stroke=\"purple\" stroke-width=\"2\" marker-start=\"url(#Arrow1Sstart)\" />"));
  }
}
} // namespace turtlelib