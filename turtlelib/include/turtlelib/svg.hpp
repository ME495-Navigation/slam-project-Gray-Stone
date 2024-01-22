#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Generate svg from 2D transformation and geometries

#include <iosfwd> // contains forward definitions for iostream objects

#include <turtlelib/se2d.hpp>

#include <filesystem>

namespace turtlelib {

//! @brief Generate svg objects for geometry2 and se2 objects.
//! The class doesn't have internal storage. It's up to caller to concatenate
//! them
class Svg {
public:
  void AddObject(Point2D point, std::string color,
                 Transform2D base_frame = {{0, 0}, 0});
  void AddObject(Vector2D vec, std::string color,
                 Transform2D base_frame = {{0, 0}, 0});
  void AddObject(Transform2D tf, std::string name,
                 Transform2D base_frame = {{0, 0}, 0});
  friend std::ostream &operator<<(std::ostream &os, const Svg &svg);

private:
  // internal storage of added objects.
  std::string constructed_objects;
};
std::ostream &operator<<(std::ostream &os, const Svg &svg);
} // namespace turtlelib

#endif