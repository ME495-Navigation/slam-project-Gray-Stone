

#include "turtlelib/svg.hpp"
#include <ostream>
#include <sstream>
#include <string>
// Design:
// Every element will need to specify a frame it's in (A transform).
// All element will be wrapped in a <g> tag which will apply its base transform.

// Drawable items includes:
// dot, array-line, frame (with name)

namespace {

    const double kScaleUp = 96; // why not make these constexpr?  // can use auto here
const unsigned int kPageWidth = 816;
const unsigned int kPageHeight = 1056;

//! @brief Warp the content with a <g> group tag with it's translation and
//! rotation set to the given tf
//! @param content string of svg contents to be wrapped in
//! @param wrapping_tf the transform that's wrapping the content in
//! @return The wrapped string
std::string WarpWithGroupTransform(std::string content,
                                   turtlelib::Transform2D wrapping_tf) {
  std::stringstream ss;
  ss << "<g transform=\"translate(" << wrapping_tf.translation().x* kScaleUp << ","
     << wrapping_tf.translation().y* kScaleUp << ") rotate("
     << turtlelib::rad2deg(wrapping_tf.rotation()) << ")\" >\n";
  ss << content << "</g>\n";
  return ss.str();
}

} // namespace

namespace turtlelib {

// All elements are added with native coordinate. Moving to page center and
// flipping Y are done through a overall <g translate></g> group tag. ONLY the
// text needs to be pre-flipped

void Svg::AddObject(Point2D point, std::string color, Transform2D base_frame) {
  std::stringstream ss;
  ss << "  <circle cx=\"" << point.x *kScaleUp << "\" cy=\"" << point.y * kScaleUp 
     << "\" r=\"3\" stroke=\"" << color << "\" fill=\"" << color
     << "\" stroke-width=\"1\" />\n";
  constructed_objects += "\n\n" + WarpWithGroupTransform(ss.str(), base_frame);
}

void Svg::AddObject(Vector2D vec, std::string color, Transform2D base_frame) {
  std::stringstream ss;

  ss << "  <line x1=\"" << vec.x * kScaleUp << "\" y1=\"" << vec.y * kScaleUp << "\" stroke=\""
     << color
     << "\" stroke-width=\"2\" "
        "marker-start=\"url(#Arrow1Sstart)\" />\n";
  constructed_objects += "\n\n" + WarpWithGroupTransform(ss.str(), base_frame);
}

void Svg::AddObject(Transform2D tf, std::string name, Transform2D base_frame) {

  std::stringstream ss;

  // Since we are flipping everything around at the end, Text is the special
  // item needs to be "pre-flipped"

  ss << "<line x1=\""<< 1*kScaleUp <<"\" stroke=\"red\" stroke-width=\"2\" "
        "marker-start=\"url(#Arrow1Sstart)\" />\n"
        "<line y1=\""<<1*kScaleUp<<"\" stroke=\"green\" stroke-width=\"2\" "
        "marker-start=\"url(#Arrow1Sstart)\" />\n"
        "<text x=\"5\" y=\"-10\" font-size=\"10\" transform=\"scale(1,-1)\" >{"
     << name << "}</text>\n";
  constructed_objects +=
      "\n\n" +
      WarpWithGroupTransform(WarpWithGroupTransform(ss.str(), tf), base_frame);
}

std::ostream &operator<<(std::ostream &os, const Svg &svg) {
    // no commented out code
//   os << "<svg width=\"" << kPageWidth << "\" height=\"" << kPageHeight
//      << "\" viewBox=\"0 0 " << kPageWidth << " " << kPageHeight
//      << "\" "


     os << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n"
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
        "</defs>\n\n"
     << "<g transform=\"translate(" << kPageWidth / 2 << "," << kPageHeight / 2
     << ") scale(1,-1)\" >\n"
     << svg.constructed_objects << "</g>\n"
     << "\n</svg>\n";

  return os;
}

} // namespace turtlelib
