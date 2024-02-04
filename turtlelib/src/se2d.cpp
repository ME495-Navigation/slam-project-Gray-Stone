#include "turtlelib/se2d.hpp"

#include <sstream>

#include <cmath>
#include <iostream>

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
    : x(trans.x), y(trans.y), cos_t(cos(radians)), sin_t(sin(radians)) {}

Point2D Transform2D::operator()(Point2D p) const {
  return {
      cos_t * p.x - sin_t * p.y + x,
      sin_t * p.x + cos_t * p.y + y,
  };
}
Vector2D Transform2D::operator()(Vector2D v) const {

  // This is the calculator I used to get the equation out
  // https://www.symbolab.com/solver/step-by-step/%5Cbegin%7Bpmatrix%7Dc_%7Bt%7D%26-s_%7Bt%7D%26x%5C%5C%20%20%20s_%7Bt%7D%26c_%7Bt%7D%26y%5C%5C%20%20%200%260%261%5Cend%7Bpmatrix%7D%5Ccdot%5Cbegin%7Bpmatrix%7Dv_%7Bx%7D%5C%5C%20%20%20v_%7By%7D%5C%5C%20%20%200%5Cend%7Bpmatrix%7D?or=input
  return {cos_t * v.x - sin_t * v.y, sin_t * v.x + cos_t * v.y};
}

Twist2D Transform2D::operator()(Twist2D v) const {
  // Original
  // row1[0, 1, 2]
  // row2[0, 1, 2]
  // [ 0, 0, 1]
  // Adjoint
  // [1, 0, 0,]
  // [y cos -sin]
  // [-x sin cos]

  // The calculator for the math
  // https://www.symbolab.com/solver/step-by-step/%5Cbegin%7Bpmatrix%7D1%260%260%5C%5C%20%20%20y_%7Bf%7D%26c_%7BT%7D%26-s_%7Bt%7D%5C%5C%20%20%20-x_%7Bf%7D%26s_%7Bt%7D%26c_%7Bt%7D%5Cend%7Bpmatrix%7D%5Cbegin%7Bpmatrix%7D%5Comega%5C%5C%20%20%20v_%7Bx%7D%5C%5C%20%20%20%20v_%7By%7D%20%20%20%20%5Cend%7Bpmatrix%7D?or=input
  // So adjoint * v becomes:
  return {v.omega, v.omega * y + cos_t * v.x - sin_t * v.y,
          -v.omega * x + sin_t * v.x + cos_t * v.y};
}

Transform2D Transform2D::inv() const {
  return Transform2D{Vector2D{-x * cos_t - y * sin_t, -y * cos_t + x * sin_t},
                     -rotation()};
}

Transform2D &Transform2D::operator*=(const Transform2D &rhs) {
  // This is source of me math
  // https://www.symbolab.com/solver/step-by-step/%5Cbegin%7Bpmatrix%7Dc_%7Bt%7D%26-s_%7Bt%7D%26x%5C%5C%20%20%20%20s_%7Bt%7D%26c_%7Bt%7D%26y%5C%5C%20%20%20%200%260%261%5Cend%7Bpmatrix%7D%5Ccdot%5Cbegin%7Bpmatrix%7Dc_%7B2%7D%26-s_%7B2%7D%26x_%7B2%7D%5C%5C%20%20%20%20%20s_%7B2%7D%26c_%7B2%7D%26y_%7B2%7D%5C%5C%20%20%20%20%200%260%261%5Cend%7Bpmatrix%7D?or=input
  Point2D p_rhs{rhs.x, rhs.y};
  auto new_p = (*this)(p_rhs);
  x = new_p.x;
  y = new_p.y;
  auto new_cos_t = cos_t * rhs.cos_t - sin_t * rhs.sin_t;
  auto new_sin_t = cos_t * rhs.sin_t + sin_t * rhs.cos_t;
  cos_t = new_cos_t;
  sin_t = new_sin_t;

  return *this;
}

Vector2D Transform2D::translation() const { return {x, y}; }

double Transform2D::rotation() const { return atan2(sin_t, cos_t); }

bool almost_equal(const Transform2D &tf1, const Transform2D &tf2,
                  double epsilon) {
  if (!almost_equal(tf1.translation(), tf2.translation(), epsilon)) {
    return false;
  }
  if (!almost_equal(tf1.rotation(), tf2.rotation(), epsilon)) {
    return false;
  }
  return true;
}

Transform2D integrate_twist(Twist2D twist) {
  if (almost_equal(twist.omega, 0)) {
    return Transform2D(Vector2D{twist.x, twist.y});
  }
  // See equation 6 and 7 in kinematics.md
  // {b} is original body frame.
  // {s} is CoR frame

  // Find center of rotation CoR from twist:
  Vector2D p_sb{twist.y / twist.omega, -twist.x / twist.omega};

  // Rotate this CoR frame by omega, which is Tbs'
  Transform2D Tbs = Transform2D{p_sb}.inv();

  // Transform2D Tbs_prime{p_sb,twist.omega};
  Transform2D Tss_prime{twist.omega};

  // Ts'b' = Tb's' ^ -1
  // Transform2D Ts_primt_b_primt = Transform2D{p_sb};
  Transform2D Ts_primt_b_primt = Tbs.inv();

  // Tbb' = Tbs' * Ts'b'
  // return Tbs_prime * Ts_primt_b_primt ;
  return Tbs * Tss_prime * Ts_primt_b_primt;
}

std::ostream &operator<<(std::ostream &os, const Transform2D &tf) {
  os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.x << " y: " << tf.y;
  return os;
}

std::istream &operator>>(std::istream &is, Transform2D &tf) {
  double deg;
  Vector2D vec;

  if (is.peek() == 'd') {
    // Get until colon
    is.ignore(10, ':');
    is >> deg;
    is.ignore(10, ':');
    is >> vec.x;
    is.ignore(10, ':');
    is >> vec.y;
  } else {
    // Second case where user directly put in all 3 numbers
    is >> deg >> vec.x >> vec.y;
  }
  tf = Transform2D{vec, deg2rad(deg)};
  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D &rhs) {
  Transform2D new_t = lhs;
  new_t *= rhs;
  return new_t;
}

} // namespace turtlelib