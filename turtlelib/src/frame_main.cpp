#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <fstream>
#include <iostream>
namespace {
//! @brief read from std cin with error checking (looped retry) and provides a
//! prompt
//! @tparam T type of the output object
//! @param prompt prompt to display before reading cin
//! @return the object T constructed using input
template <typename T> T ErrorCheckPromptedInput(std::string prompt) {
  // Citation ---------- [3] ----------
  T out;
  std::cout << prompt;
  std::cin >> out;
  while (std::cin.fail()) {
    // Input error happened
    if (std::cin.eof()) {
      std::cout << "Received EOF!, exiting";
      exit(0);
    }
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // https://stackoverflow.com/questions/28635130/input-error-checking-in-c
    std::cout << "Input error, try again\n" << prompt;
    std::cin >> out;
  }
  return out;
  // End of citation ---------- [3] ----------
}
} // namespace

using namespace turtlelib;
int main() {

  // Asking for 2 transforms

  // Prompt the user to enter two transforms: Tab and Tbc.

  Transform2D Tab = ErrorCheckPromptedInput<Transform2D>(
      "Input Transform T_{ab} in form of <x> <y> <deg> \n");

  Transform2D Tbc = ErrorCheckPromptedInput<Transform2D>(
      "Input Transform T_{bc} in form of <x> <y> <deg> \n");

  // Compute and output Tab, Tba, Tbc, Tcb, Tac, and Tca

  Transform2D Tba = Tab.inv();
  Transform2D Tcb = Tbc.inv();
  Transform2D Tac = Tab * Tbc;
  Transform2D Tca = Tac.inv();

  std::cout << "T_{ab} : " << Tab << "\n";
  std::cout << "T_{ba} : " << Tba << "\n";
  std::cout << "T_{bc} : " << Tbc << "\n";
  std::cout << "T_{cb} : " << Tcb << "\n";
  std::cout << "T_{ac} : " << Tac << "\n";
  std::cout << "T_{ca} : " << Tca << "\n";

  // and draw each frame in the svg file (with frame A located at (0, 0)).
  Svg svg;

  svg.AddObject(Transform2D(), "A");
  svg.AddObject(Tab, "B");
  svg.AddObject(Tac, "C");

  // Prompt the user to enter a point p_a in Frame {a}
  Point2D p_a =
      ErrorCheckPromptedInput<Point2D>("Enter a point p_a in Frame {a}\n");

  // Compute p_a's location in frames b and c and output the locations of all 3
  // points
  Point2D p_b = Tba(p_a);
  Point2D p_c = Tca(p_a);

  std::cout << "p_a: " << p_a << "\n";
  std::cout << "p_b: " << p_b << "\n";
  std::cout << "p_c: " << p_c << "\n";

  //     Use purple to draw p_a, brown to draw p_b, and orange to draw p_c.
  // TODO(LEO) what frame to put these in?
  svg.AddObject(p_a, "purple");
  svg.AddObject(p_b, "brown");
  svg.AddObject(p_c, "orange");

  // Prompt the user to enter a vector vb in frame b

  Vector2D vb =
      ErrorCheckPromptedInput<Vector2D>("Enter a vector vb in Frame {b}\n");

  //     Normalize the vector to form v^b.
  auto vb_hat = vb.normalize();

  std::cout << "v_b_hat: " << vb_hat << "\n";
  // Draw v^b with the tail located at (0,0) in frame b, in brown.
  svg.AddObject(vb_hat, "brown", Tab);

  // Output vb expressed in frame a and frame c coordinates

  Vector2D va = Tba(vb);
  Vector2D vc = Tbc(vb);
  std::cout << "vb in frame a, v_a: " << va << "\n";
  std::cout << "vb in frame b, v_b: " << vb << "\n";
  std::cout << "vb in frame c, v_c: " << vc << "\n";

  // Draw va with the tail at (0,0) in frame a in purple.
  svg.AddObject(va, "purple");
  // Draw vc with the tail at (0,0 in frame c in orange
  svg.AddObject(vc, "orange");

  // TODO this section is in example output, but not in description
  // Twist2D twist_b = ErrorCheckPromptedInput<Twist2D>("Enter twist V_b\n");

  Twist2D twist_b;
  std::cout << "Enter twist V_b\n";
  // TODO, This is simply not working right!
  std::cin >> twist_b;

  Twist2D twist_a = Tba(twist_b);
  Twist2D twist_c = Tca(twist_b);

  std::cout << "Twist_a : " << twist_a << "\n";
  std::cout << "Twist_b : " << twist_b << "\n";
  std::cout << "Twist_c : " << twist_c << "\n";

  // Output the drawing to /tmp/frames.svg.

  std::ofstream out_stream("/tmp/frames.svg");

  out_stream << svg;
  return 0;
}