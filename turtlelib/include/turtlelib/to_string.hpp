
#ifndef TURTLELIB_TO_STRING_INCLUDE_GUARD_HPP
#define TURTLELIB_TO_STRING_INCLUDE_GUARD_HPP

#include <sstream>

#include <string>

namespace turtlelib {
// ############# Begin Citation [5]#############

class ToString {
public:
  ToString() {}
  ~ToString() {}

  template <typename Type> ToString &operator<<(const Type &value) {
    stream_ << value;
    return *this;
  }

  std::string str() const { return stream_.str(); }
  operator std::string() const { return stream_.str(); }

  enum ConvertToString { to_str };
  std::string operator>>(ConvertToString) { return stream_.str(); }

private:
  std::stringstream stream_;

  ToString(const ToString &);
  ToString &operator=(ToString &);
};

// ############# End Citation [5]#############
} // namespace turtlelib

#endif
