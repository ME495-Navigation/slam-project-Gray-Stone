#include "turtlelib/se2d.hpp"

#include <sstream>

#include <cmath>

namespace turtlelib {
std::ostream &operator<<(std::ostream &os, const Twist2D &tw) {
  os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
  return os;
}

std::istream &operator>>(std::istream &is, Twist2D &tw) {
  if (is.peek() == '[') {
    is.get();
  }
  is >> tw.omega >> tw.x >> tw.y;

  if (is.peek() == ']') {
    is.get();
  }
  return is;
}

// Transform2D classes

// Transform2D::Transform2D() : row1{1.0, 0.0, 0.0}, row2{0.0, 1.0, 0.0} {}
Transform2D::Transform2D() : Transform2D({.0, .0}, .0) {}

Transform2D::Transform2D(Vector2D trans) : Transform2D(trans, 0.0) {}
Transform2D::Transform2D(double radians) : Transform2D({0, 0}, radians) {}
Transform2D::Transform2D(Vector2D trans, double radians)
    : row1{cos(radians), -sin(radians), trans.x},
      row2{sin(radians), cos(radians), trans.y} {}

Vector2D Transform2D::translation() const { return {row1[2], row2[2]}; }

double Transform2D::rotation() const { return atan2(row2[0], row1[0]); }

Point2D Transform2D::operator()(Point2D p) const {
  return {
      row1[0] * p.x + row1[1] * p.x + row1[2],
      row2[0] * p.y + row2[1] * p.y + row2[2],
  };
}
Vector2D Transform2D::operator()(Vector2D v) const {
  return {
      row1[0] * v.x + row1[1] * v.x + row1[2],
      row2[0] * v.y + row2[1] * v.y + row2[2],
  };
}

Twist2D Transform2D::operator()(Twist2D v) const {
  // Original
  // row1[0, 1, 2]
  // row2[0, 1, 2]
  // [ 0, 0, 1]
  // Adjoint
  // [1, 0, 0,]
  // [y cos -sin]
  // [x sin cos]
  // which is also
  // [1,0,0]
  // [row2[2] row1[0] row1[1]]
  // [-row1[2] row2[0] row2[1] ]

  // So adjoint * v becomes:
  return {v.omega, row2[2] * v.omega + row1[0] * v.x + row1[1] * v.y,
          -row1[2] * v.omega + row2[0] * v.x + row2[1] * v.y};
}

Transform2D Transform2D::inv() const {
  return Transform2D{Vector2D{-row1[2] * row1[0] - row2[2] * row2[0],
                              -row2[2] * row2[0] + row1[2] * row2[0]},
                     rotation()};
}

} // namespace turtlelib