#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.

#include <cmath>
#include <iosfwd> // contains forward definitions for iostream objects

namespace turtlelib {
/// \brief PI.  Not in C++ standard until C++20.
constexpr double PI = 3.14159265358979323846;

/// \brief approximately compare two floating-point numbers using
///        an absolute comparison
/// \param d1 - a number to compare
/// \param d2 - a second number to compare
/// \param epsilon - absolute threshold required for equality
/// \return true if abs(d1 - d2) < epsilon
/// NOTE: implement this in the header file
/// constexpr means that the function can be computed at compile time
/// if given a compile-time constant as input
constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12) {
  return std::abs(d1 - d2) < epsilon;
}

/// \brief convert degrees to radians
/// \param deg - angle in degrees
/// \returns radians
constexpr double deg2rad(double deg) { return (deg / 180) * PI; }

/// \brief convert radians to degrees
/// \param rad - angle in radians
/// \returns the angle in degrees
constexpr double rad2deg(double rad) { return (rad / PI) * 180; }

/// \brief wrap an angle to (-PI, PI]
/// \param rad (angle in radians)
/// \return an angle equivalent to rad but in the range (-PI, PI]
double normalize_angle(double rad);

/// static_assertions test compile time assumptions.
/// You should write at least one more test for each function
/// You should also purposely (and temporarily) make one of these tests fail
/// just to see what happens
static_assert(almost_equal(0, 0), "is_zero failed");

static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

/// \brief a 2-Dimensional Point
struct Point2D {
  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;
};

/// \brief output a 2 dimensional point as [xcomponent ycomponent]
/// \param os - stream to output to
/// \param p - the point to print
std::ostream &operator<<(std::ostream &os, const Point2D &p);

/// \brief input a 2 dimensional point
///   You should be able to read vectors entered as follows:
///   [x y] or x y
/// \param is - stream from which to read
/// \param p [out] - output vector
/// HINT: See operator>> for Vector2D
std::istream &operator>>(std::istream &is, Point2D &p);

/// \brief A 2-Dimensional Vector
struct Vector2D {
  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;

  //! @brief get the normalized version of this vector
  //! @return a Vector2D that is normalized
  Vector2D normalize() const;

  //! @brief get the magnitude of this vector
  //! @return magnitude in double
  double magnitude() const;

  //! @brief Self increment operator, add current vector by given vector
  //! @param rhs Vector to add to the back
  //! @return This vector itself.
  Vector2D &operator+=(const Vector2D &rhs);

  //! @brief Self decrement operator, subtract current vector by given vector
  //! @param rhs Vector to subtract to the back
  //! @return This vector itself.
  Vector2D &operator-=(const Vector2D &rhs);

  //! @brief scale this vector by a scaler.
  //! @param rhs scaler to scale the vector.
  //! @return This vector itself.
  Vector2D &operator*=(const double &rhs);
};

/// \brief Subtracting one point from another yields a vector
/// \param head point corresponding to the head of the vector
/// \param tail point corresponding to the tail of the vector
/// \return a vector that points from p1 to p2
/// NOTE: this is not implemented in terms of -= because subtracting two Point2D
/// yields a Vector2D
Vector2D operator-(const Point2D &head, const Point2D &tail);

/// \brief Adding a vector to a point yields a new point displaced by the vector
/// \param tail The origin of the vector's tail
/// \param disp The displacement vector
/// \return the point reached by displacing by disp from tail
/// NOTE: this is not implemented in terms of += because of the different types
Point2D operator+(const Point2D &tail, const Vector2D &disp);

/// \brief output a 2 dimensional vector as [xcomponent ycomponent]
/// \param os - stream to output to
/// \param v - the vector to print
std::ostream &operator<<(std::ostream &os, const Vector2D &v);

/// \brief input a 2 dimensional vector
///   You should be able to read vectors entered as follows:
///   [x y] or x y
/// \param is - stream from which to read
/// \param v [out] - output vector
/// Hint: The following may be useful:
/// https://en.cppreference.com/w/cpp/io/basic_istream/peek
/// https://en.cppreference.com/w/cpp/io/basic_istream/get
///
/// The way input works is (more or less): what the user types is stored in a
/// buffer until the user enters a newline (by pressing enter).  The iostream
/// methods then process the data in this buffer character by character.
/// Typically, each character is examined and then removed from the buffer
/// automatically. If the characters don't match what is expected (e.g., we are
/// expecting an int but the letter 'q' is encountered) an error flag is set on
/// the stream object (e.g., std::cin).
///
/// We have lower level control however. For example:
/// peek looks at the next unprocessed character in the buffer without removing
/// it get removes the next unprocessed character from the buffer.
std::istream &operator>>(std::istream &is, Vector2D &v);

//! @brief Add two vector together.
//! @param lhs Vector 1
//! @param rhs Vector 2
//! @return sum of two vector
Vector2D operator+(const Vector2D &lhs, const Vector2D &rhs);

//! @brief Sub track two vector
//! @param lhs Vector in front
//! @param rhs Vector in back
//! @return result of subtraction
Vector2D operator-(const Vector2D &lhs, const Vector2D &rhs);

//! @brief scale the vector by a scaler
//! @param lhs the scaler
//! @param rhs the vector
//! @return scaled vector
Vector2D operator*(const double &lhs, const Vector2D &rhs);

//! @brief scale the vector by a scaler
//! @param lhs the vector
//! @param rhs the scaler
//! @return scaled vector
Vector2D operator*(const Vector2D &lhs, const double &rhs);

//! @brief dot product of two vector.
//! @param v1 first vector
//! @param v2 second vector
//! @return dot product between two inputs
double dot(const Vector2D &v1, const Vector2D &v2);

//! @brief Get the directional angle from vector 1 to vector 2
//! @param v1 Starting vector
//! @param v2 Ending vector
//! @return Angle from starting vector to ending vector. output is from atan2
//! (signed, within -PI PI)
double angle(const Vector2D &v1, const Vector2D &v2);

constexpr bool almost_equal(const Vector2D &v1, const Vector2D &v2,
                            double epsilon = 1.0e-12) {
  if (!almost_equal(v1.x, v2.x, epsilon)) {
    return false;
  }

  if (!almost_equal(v1.y, v2.y, epsilon)) {
    return false;
  }
  return true;
}

static_assert(almost_equal(Vector2D{3.0, 1.0}, Vector2D{3.0, 1.0}),
              "Vector almost_equal failed");

// bool almost_equal(const Vector2D &v1, const Vector2D &v2,
//                   double epsilon = std::numeric_limits<double>::epsilon() *
//                                    100);
} // namespace turtlelib

#endif
