#include "turtlelib/test_utils.hpp"
#include <sstream>
namespace turtlelib {

Vector2DWithinRel::Vector2DWithinRel(Vector2D const &target_vec, double rel)
    : vec{target_vec}, rel{rel} {}

bool Vector2DWithinRel::match(Vector2D const &in) const {
  if (!WithinRel(vec.x, rel).match(in.x)) {
    return false;
  }
  if (!WithinRel(vec.y, rel).match(in.y)) {
    return false;
  }
  return true;
}

std::string Vector2DWithinRel::describe() const {
  std::ostringstream os;
  os << "and " << vec << " are within " << rel << " of each other";
  return os.str();
}

Transform2DWithinRel::Transform2DWithinRel(Transform2D const &target_tf,
                                           double rel)
    : tf{target_tf}, rel{rel} {}

bool Transform2DWithinRel::match(Transform2D const &in) const {

  if (!Vector2DWithinRel(tf.translation(), rel).match(in.translation())) {
    return false;
  }
  if (!WithinRel(tf.rotation(), rel).match(in.rotation())) {
    return false;
  }
  return true;
}

std::string Transform2DWithinRel::describe() const {
  std::ostringstream os;
  os << "and " << tf << " are within " << rel << " of each other";
  return os.str();
}

} // namespace turtlelib