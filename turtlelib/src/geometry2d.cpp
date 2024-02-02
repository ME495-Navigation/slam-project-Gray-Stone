#include "turtlelib/geometry2d.hpp"
#include <iostream>
#include <string>

namespace turtlelib {

double normalize_angle(double rad) {
  // Citation ---------- [2] ----------
  return rad - (std::ceil((rad + PI) / (2 * PI)) - 1) * 2 * PI;
}
std::ostream &operator<<(std::ostream &os, const Point2D &p) {
  os << "[" << p.x << " " << p.y << "]";
  return os;
}

std::istream &operator>>(std::istream &is, Point2D &p) {
  if (is.peek() == '[') {
    is.get();
  }
  is >> p.x >> p.y;

  if (is.peek() == ']') {
    is.get();
  }

  return is;
}

Vector2D Vector2D::normalize() const {
  double norm = magnitude();
  return {x / norm, y / norm};
}

double Vector2D::magnitude() const{
    return sqrt(x*x + y*y);
}

Vector2D operator-(const Point2D &head, const Point2D &tail) {
  return {head.x - tail.x, head.y - tail.y};
}

Point2D operator+(const Point2D &tail, const Vector2D &disp) {
  return {tail.x + disp.x, tail.y + disp.y};
}

std::ostream &operator<<(std::ostream &os, const Vector2D &v) {
  os << "[" << v.x << " " << v.y << "]";
  return os;
}

std::istream &operator>>(std::istream &is, Vector2D &v) {
  if (is.peek() == '[') {
    is.get();
  }
  is >> v.x >> v.y;

  if (is.peek() == ']') {
    is.get();
  }

  return is;
}

Vector2D &Vector2D::operator+=(const Vector2D &rhs) {
  x += rhs.x;
  y += rhs.y;
  return *this;
}
Vector2D &Vector2D::operator-=(const Vector2D &rhs) {
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}
Vector2D &Vector2D::operator*=(const double &rhs) {
  x *= rhs;
  y *= rhs;
  return *this;
}

Vector2D operator+(const Vector2D &lhs, const Vector2D &rhs) {

  return {
      lhs.x + rhs.x,
      lhs.y + rhs.y,
  };
}

Vector2D operator-(const Vector2D &lhs, const Vector2D &rhs) {

  return {
      lhs.x - rhs.x,
      lhs.y - rhs.y,
  };
}
Vector2D operator*(const Vector2D &lhs, const double &rhs) {

  return {
      lhs.x * rhs,
      lhs.y * rhs,
  };
}
Vector2D operator*(const double &lhs, const Vector2D &rhs) { return rhs * lhs; }

double dot(const Vector2D& v1, const Vector2D& v2){
  return v1.x * v2.x + v1.y * v2.y;
}


double angle(const Vector2D &v1, const Vector2D &v2){

  // Citation ----------[4]---------- 
  //     dot = x1*x2 + y1*y2      # Dot product between [x1, y1] and [x2, y2]
  // det = x1*y2 - y1*x2      # Determinant
  double det = v1.x*v2.y - v1.y * v2.x ;
  // angle = atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
  return atan2( det , dot(v1, v2));
}


} // namespace turtlelib