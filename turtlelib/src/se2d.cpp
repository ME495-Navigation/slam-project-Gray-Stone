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
    : x(trans.x), y(trans.y), cos_t(cos(radians)), sin_t(sin(radians)) {}
// Transform2D::Transform2D(Vector2D trans, double radians)
//     : row1{cos(radians), -sin(radians), trans.x},
//       row2{sin(radians), cos(radians), trans.y} {}


Point2D Transform2D::operator()(Point2D p) const {
  return {
      cos_t * p.x - sin_t * p.y + x,
      sin_t * p.x + cos_t * p.y + y,
  };
  // return {
  //     row1[0] * p.x + row1[1] * p.x + row1[2],
  //     row2[0] * p.y + row2[1] * p.y + row2[2],
  // };
}
Vector2D Transform2D::operator()(Vector2D v) const {

  // This is the calculator I used to get the equation out
  // https://www.symbolab.com/solver/step-by-step/%5Cbegin%7Bpmatrix%7Dc_%7Bt%7D%26-s_%7Bt%7D%26x%5C%5C%20%20%20s_%7Bt%7D%26c_%7Bt%7D%26y%5C%5C%20%20%200%260%261%5Cend%7Bpmatrix%7D%5Ccdot%5Cbegin%7Bpmatrix%7Dv_%7Bx%7D%5C%5C%20%20%20v_%7By%7D%5C%5C%20%20%200%5Cend%7Bpmatrix%7D?or=input
  return {cos_t * v.x - sin_t * v.y, sin_t * v.x + cos_t * v.y};

  // return {
  //     row1[0] * v.x + row1[1] * v.x + row1[2],
  //     row2[0] * v.y + row2[1] * v.y + row2[2],
  // };
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
  // https://www.symbolab.com/solver/step-by-step/%5Cbegin%7Bpmatrix%7D1%260%260%5C%5C%20y%26c_%7Bt%7D%26-s_%7Bt%7D%5C%5C%20-x%26s_%7Bt%7D%26c_%7Bt%7D%5Cend%7Bpmatrix%7D%5Cbegin%7Bpmatrix%7D%5Comega%5C%5C%20v_%7Bx%7D%5C%5C%20%20v_%7By%7D%20%20%20%20%5Cend%7Bpmatrix%7D?or=input

  // So adjoint * v becomes:
  return {v.omega, v.omega * x + cos_t * v.y - sin_t * v.x,
          -v.omega * y + sin_t * v.y + cos_t * v.x};
}

Transform2D Transform2D::inv() const {
  return Transform2D{Vector2D{-row1[2] * row1[0] - row2[2] * row2[0],
                              -row2[2] * row2[0] + row1[2] * row2[0]},
                     rotation()};
}

  Transform2D &Transform2D::operator*=(const Transform2D &rhs){
    
    // This is source of me math
    // https://www.symbolab.com/solver/step-by-step/%5Cbegin%7Bpmatrix%7Dc_%7Bt%7D%26-s_%7Bt%7D%26x%5C%5C%20%20%20%20s_%7Bt%7D%26c_%7Bt%7D%26y%5C%5C%20%20%20%200%260%261%5Cend%7Bpmatrix%7D%5Ccdot%5Cbegin%7Bpmatrix%7Dc_%7B2%7D%26-s_%7B2%7D%26x_%7B2%7D%5C%5C%20%20%20%20%20s_%7B2%7D%26c_%7B2%7D%26y_%7B2%7D%5C%5C%20%20%20%20%200%260%261%5Cend%7Bpmatrix%7D?or=input
    Point2D p_rhs{rhs.x,rhs.y};
    auto new_p = (*this)(p_rhs);
    x = new_p.x;
    y = new_p.y;
    auto new_cos_t = cos_t*rhs.cos_t - sin_t*rhs.sin_t;
    auto new_sin_t = cos_t*rhs.sin_t + sin_t*rhs.cos_t;
    cos_t = new_cos_t;
    sin_t = new_sin_t;

    return *this;
  }

Vector2D Transform2D::translation() const { return {x, y}; }
// Vector2D Transform2D::translation() const { return {row1[2], row2[2]}; }

// double Transform2D::rotation() const { return atan2(row2[0], row1[0]); }
double Transform2D::rotation() const { return atan2(sin_t, cos_t); }


std::ostream &operator<<(std::ostream &os, const Transform2D &tf){
  os << "x: "<< tf.x << " y: " << tf.y << " deg: " << rad2deg(tf.rotation());
  return os; 
}

std::istream &operator>>(std::istream &is, Transform2D &tf){
  double deg;
  Vector2D vec; 
  
  is >> vec.x >> vec.y >> deg;
  tf = Transform2D{vec,deg};
  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D &rhs){
    Point2D p_rhs{rhs.translation().x,rhs.translation().y};
    auto new_p = lhs(p_rhs);

    double new_ang = lhs.rotation() + rhs.rotation();
    return {{new_p.x , new_p.y},new_ang};

}


} // namespace turtlelib