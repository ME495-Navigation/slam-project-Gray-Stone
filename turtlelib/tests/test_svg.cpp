#include "turtlelib/svg.hpp"

#include <catch2/catch_test_macros.hpp>

#include <catch2/matchers/catch_matchers_string.hpp>

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::EndsWith;
using Catch::Matchers::Equals;
using Catch::Matchers::StartsWith;

namespace turtlelib {

TEST_CASE("svg test ", "[svg]") {
  Svg svg;

  std::ostringstream oss;

  SECTION("Svg file header", "[svg]") {

    // Note the ending new line, This matters
    oss << svg;
    REQUIRE_THAT(oss.str(), StartsWith("<svg") && EndsWith("</svg>\n"));
  }

  SECTION("Point Object test", "[svg]") {
    svg.AddObject(Point2D{10, -10}, "blue");
    oss << svg;
    REQUIRE_THAT(oss.str(),
                 ContainsSubstring(
                     "<circle cx=\"960\" cy=\"-960\" r=\"3\" stroke=\"blue\" "
                     "fill=\"blue\" stroke-width=\"1\" />"));
  }

  SECTION("Vector2D Object test", "[svg]") {
    svg.AddObject(Vector2D{-5, 9}, "purple");
    oss << svg;

    REQUIRE_THAT(
        oss.str(),
        ContainsSubstring(
            "<line x1=\"-480\" y1=\"864\" stroke=\"purple\" stroke-width=\"2\" "
            "marker-start=\"url(#Arrow1Sstart)\" />"));
  }

  SECTION("Frame Object test", "[svg]") {
    Transform2D frame_1{{50, 50}, deg2rad(22.2)};

    svg.AddObject(frame_1, "some_name");
    oss << svg;

    REQUIRE_THAT(
        oss.str(),
        ContainsSubstring(
            R"xxx(<g transform="translate(0,0) rotate(0)" >)xxx"
            "\n"
            R"xxx(<g transform="translate(4800,4800) rotate(22.2)" >)xxx"
            "\n"
            R"xxx(<line x1="96" stroke="red" stroke-width="2" marker-start="url(#Arrow1Sstart)" />)xxx"
            "\n"
            R"xxx(<line y1="96" stroke="green" stroke-width="2" marker-start="url(#Arrow1Sstart)" />)xxx"
            "\n"
            R"xxx(<text x="5" y="-10" font-size="10" transform="scale(1,-1)" >{some_name}</text>)xxx"
            "\n"
            R"xxx(</g>)xxx"
            "\n"
            R"xxx(</g>)xxx"
            "\n"));
  }
}
} // namespace turtlelib
