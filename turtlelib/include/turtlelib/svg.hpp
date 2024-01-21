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
  std::string WarpFileHeader(std::string contents);
  std::ostream& FinishAndWriteToFile(std::ostream &out_stream, std::string content);
  std::string MakeObject(Point2D point);
  std::string MakeObject(Vector2D vec);
  std::string MakeFrame(Transform2D tf, std::string name);

  //! @brief Generate a <g> group tag with it's translation and rotation set to
  //! the given tf
  //! @param content content inside the <g> group
  //! @param wrapping_tf The transform for the group to be at
  //! @return given content wrapped in group <g>
  std::string WarpWithGroupTransform(std::string content,
                                     Transform2D wrapping_tf);
};

} // namespace turtlelib

#endif