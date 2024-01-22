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
  //! @brief Add a point object to svg
  //! @param point Point2D object to be added
  //! @param color The color of the object
  //! @param base_frame The frame this Point2D object is in (default to center,
  //! 0 angle)
  void AddObject(Point2D point, std::string color,
                 Transform2D base_frame = {{0, 0}, 0});
  //! @brief Add a Vector2D object to svg
  //! @param vec Vector2D object to be added
  //! @param color Color of the vector stem
  //! @param base_frame The frame's which the vector is base off of (aka tail of
  //! vector will be at 0,0 of the frame) (default to center, 0 angle)
  void AddObject(Vector2D vec, std::string color,
                 Transform2D base_frame = {{0, 0}, 0});

  //! @brief Add a Frame object to svg
  //! @param tf The Frame's location and orientation (wrt to base_frame
  //! argument)
  //! @param name display name of the frame
  //! @param base_frame The base frame for the transform (default to center, 0
  //! angle)
  void AddObject(Transform2D tf, std::string name,
                 Transform2D base_frame = {{0, 0}, 0});
  //! @brief See operator<< outside.
  friend std::ostream &operator<<(std::ostream &os, const Svg &svg);

private:
  std::string constructed_objects; //!< internal storage of added objects.
};

//! @brief output the svg content to output stream. Could directly output to a
//! file stream for saving to disk
//! @param os output steam, (e.g. std::ofstream, std::cout)
//! @param svg the svg object itself
//! @return the output stream
std::ostream &operator<<(std::ostream &os, const Svg &svg);
} // namespace turtlelib

#endif