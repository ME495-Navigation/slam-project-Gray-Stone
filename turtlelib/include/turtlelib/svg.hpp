#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Generate svg from 2D transformation and geometries

#include <iosfwd> // contains forward definitions for iostream objects

#include <turtlelib/se2d.hpp>

#include <filesystem>

namespace turtlelib {

//! @brief Generate svg files for geometry2 and se2 objects added to internal
//! storage
class Svg {
public:
  //! @brief Output internal content to a file.
  //! @param file_path Path to the output file
  //! @return True: successfully write to file. False: Anything failed
  bool WriteToFile(std::filesystem::path file_path);
  void AddObject(Point2D point,Transform2D base_frame);
  void AddObject(Vector2D vec , Transform2D base_frame);
  void AddFrame(Transform2D tf,std::string name ,Transform2D base_frame);
  std::string GetConstructedObjects();
private:
  // internal storage of added objects.
  std::string constructed_objects;
};

} // namespace turtlelib

#endif