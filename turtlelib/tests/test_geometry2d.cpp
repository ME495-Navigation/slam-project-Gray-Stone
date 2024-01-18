#include <turtlelib/geometry2d.hpp>
// #include "geometry2d.hpp"

#include <catch2/catch_test_macros.hpp>

#include <catch2/matchers/catch_matchers_floating_point.hpp>



namespace turtlelib{

using Catch::Matchers::WithinRel;

TEST_CASE("Normalizing angles"){

    // double base_angle = 3* PI /2;
    REQUIRE_THAT( normalize_angle(-PI/2) , WithinRel( 3* PI /2));
}
} 