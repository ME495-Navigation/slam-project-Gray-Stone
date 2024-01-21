

#include "turtlelib/svg.hpp"
#include <sstream>
#include <string>
// Design:
// Every element will need to specify a frame it's in (A transform).
// All element will be wrapped in a <g> tag which will apply its base transform.

// Drawable items includes:
// dot, array-line, frame (with name)

namespace {
//! @brief Generate a <g> group tag with it's translation and rotation set to
//! the given tf
//! @param tf The transform for the group to be at
//! @return The string of group header: note it's missing the closing </g>
//! bracket, caller should arrange adding that.
std::string TransformToGroupTagHeader(turtlelib::Transform2D tf) {
  std::stringstream ss;
  ss << "<g transform=\"translate(" << tf.translation().x <<"," <<tf.translation().y
     << ") rotate(" << turtlelib::rad2deg(tf.rotation()) << ")\" >\n";
  return ss.str();
}
} // namespace

namespace turtlelib {

bool Svg::WriteToFile(std::filesystem::path file_path) {}
void Svg::AddObject(Point2D point, Transform2D base_frame) {
  constructed_objects+="\n"+TransformToGroupTagHeader(base_frame);
  std::stringstream ss;
  ss << "  <circle cx=\"" << point.x << "\" cy=\"" << point.y
     << "\" r=\"3\" stroke=\"blue\" fill=\"tan\" stroke-width=\"1\" />\n";
  constructed_objects += ss.str() + "</g>";
}
void Svg::AddObject(Vector2D vec, Transform2D base_frame) {
  constructed_objects+="\n"+TransformToGroupTagHeader(base_frame);
  std::stringstream ss;

  ss << "  <line x1=\"" << vec.x << "\" y1=\"" << vec.y
     << "\" stroke=\"purple\" stroke-width=\"2\" "
        "marker-start=\"url(#Arrow1Sstart)\" />\n";
  constructed_objects += ss.str() + "</g>";
}
void Svg::AddFrame(Transform2D tf,std::string name, Transform2D base_frame) {

  constructed_objects+="\n"+TransformToGroupTagHeader(tf);
  std::stringstream ss;

<line x1="20" stroke="red" stroke-width="2" marker-start="url(#Arrow1Sstart)" />\n
<line y1="20" stroke="green" stroke-width="2" marker-start="url(#Arrow1Sstart)" />\n
<text x="5" y="20" font-size="10">{a}</text>


  constructed_objects += ss.str() + "</g>";


}

  std::string Svg::GetConstructedObjects(){
    return constructed_objects;
  }

} // namespace turtlelib