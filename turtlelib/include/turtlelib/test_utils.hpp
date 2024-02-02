#ifndef TURTLELIB_TEST_UTILS_INCLUDE_GUARD_HPP
#define TURTLELIB_TEST_UTILS_INCLUDE_GUARD_HPP

#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <catch2/matchers/catch_matchers_templated.hpp>

#include <sstream>

using Catch::Matchers::WithinRel;

namespace turtlelib {

// struct Vector2DWithinRel : Catch::Matchers::MatcherGenericBase {
class Vector2DWithinRel : public Catch::Matchers::MatcherGenericBase {
public:
  Vector2DWithinRel(Vector2D const &target_vec,
                    double rel = std::numeric_limits<double>::epsilon() * 100);

  bool match(Vector2D const &in) const;

  std::string describe() const;

private:
  Vector2D vec;
  double rel;
};

class Transform2DWithinRel : public Catch::Matchers::MatcherGenericBase {
public:
  Transform2DWithinRel(Transform2D const &target_tf,
                       double rel = std::numeric_limits<double>::epsilon() *
                                    100);

  bool match(Transform2D const &in) const;

  std::string describe() const;

private:
  Transform2D tf;
  double rel;
};

} // namespace turtlelib

#endif