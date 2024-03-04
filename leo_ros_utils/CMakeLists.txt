cmake_minimum_required(VERSION 3.8)
project(leo_ros_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlelib REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# find_package(tf2_ros REQUIRED)
find_package(Doxygen)
option(BUILD_DOCS "Build the documentation" OFF)

# *************** BUILD  
# Define the library
add_library(leo_ros_utils src/math_helper.cpp)

target_include_directories(
  leo_ros_utils
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>)
  
  # Ros librarys
  ament_target_dependencies(leo_ros_utils rclcpp geometry_msgs tf2 tf2_ros)
  target_compile_features(leo_ros_utils PUBLIC cxx_std_17)
  # Turtlelib is not a ament target, thus included in this way.
  target_link_libraries(leo_ros_utils turtlelib::turtlelib)
# *************** END BUILD 

# *************** INSTALL
# install the include files by copying the whole include directory
# For other ament cmake ros package, this includes srv, launch, config, urdf, etc.
install(DIRECTORY include/leo_ros_utils DESTINATION include)

# These three lines are how to make a ament package find-able by others. 
install(TARGETS leo_ros_utils EXPORT export_leo_ros_utils)
# The folloing line is one commonly missed. 
ament_export_include_directories("include/leo_ros_utils")
ament_export_targets(export_leo_ros_utils)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
