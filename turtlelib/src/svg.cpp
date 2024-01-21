

#include "turtlelib/svg.hpp"
#include <ostream>
#include <sstream>
#include <string>
// Design:
// Every element will need to specify a frame it's in (A transform).
// All element will be wrapped in a <g> tag which will apply its base transform.

// Drawable items includes:
// dot, array-line, frame (with name)

namespace turtlelib {

std::string Svg::MakeObject(Point2D point) {
  std::stringstream ss;
  ss << "<circle cx=\"" << point.x << "\" cy=\"" << point.y
     << "\" r=\"3\" stroke=\"blue\" fill=\"tan\" stroke-width=\"1\" />\n";
  return ss.str();
}
std::string Svg::MakeObject(Vector2D vec) {
  std::stringstream ss;

  ss << "<line x1=\"" << vec.x << "\" y1=\"" << vec.y
     << "\" stroke=\"purple\" stroke-width=\"2\" "
        "marker-start=\"url(#Arrow1Sstart)\" />\n";
  return ss.str();
}
std::string Svg::MakeFrame(Transform2D tf, std::string name) {

  std::stringstream ss;
  ss << "<line x1=\"20\" stroke=\"red\" stroke-width=\"2\" "
        "marker-start=\"url(#Arrow1Sstart)\" />\n"
        "<line y1=\"20\" stroke=\"green\" stroke-width=\"2\" "
        "marker-start=\"url(#Arrow1Sstart)\" />\n"
        "<text x=\"5\" y=\"20\" font-size=\"10\">{"
     << name << "}</text>\n";

  return WarpWithGroupTransform(ss.str(), tf);
  ;
}

std::string Svg::WarpFileHeader(std::string contents) {
  return "<svg width=\"400\" height=\"400\" viewBox=\"0 0 400 400\" "
         "xmlns=\"http://www.w3.org/2000/svg\">\n\n"

         "<defs>\n"
         "  <marker\n"
         "     style=\"overflow:visible\"\n"
         "     id=\"Arrow1Sstart\"\n"
         "     refX=\"0.0\"\n"
         "     refY=\"0.0\"\n"
         "     orient=\"auto\">\n"
         "       <path\n"
         "         transform=\"scale(0.2) translate(6)\"\n"
         "         style=\"fill-rule: evenodd; stroke-width: 1pt; fill: "
         "context-stroke; stroke: context-stroke\"\n"
         "         d=\"M 0 0 L 5 -5 L -12.5 0 L 5 5 L 0 0 z\"\n"
         "         />\n"
         "    </marker>\n"
         "</defs>\n\n" +
         contents + "\n</svg>\n";
}

std::string Svg::WarpWithGroupTransform(std::string content,
                                        Transform2D wrapping_tf) {
  std::stringstream ss;
  ss << "<g transform=\"translate(" << wrapping_tf.translation().x << ","
     << wrapping_tf.translation().y << ") rotate("
     << turtlelib::rad2deg(wrapping_tf.rotation()) << ")\" >\n";
  ss << content << "</g>";
  return ss.str();
}


std::ostream& Svg::FinishAndWriteToFile(std::ostream &out_stream , std::string content){
  out_stream << WarpFileHeader(content);
  return out_stream;
}


} // namespace turtlelib