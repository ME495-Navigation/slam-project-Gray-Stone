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
find_package(geometry_msgs REQUIRED)

# find_package(tf2_ros REQUIRED)
find_package(Doxygen)
option(BUILD_DOCS "Build the documentation" OFF)

# Define the library
add_library(leo_ros_utils src/param_helper.cpp src/math_helper.cpp)
target_include_directories(
  leo_ros_utils
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>)
  
  # Ros librarys
ament_target_dependencies(leo_ros_utils rclcpp geometry_msgs tf2)
target_compile_features(leo_ros_utils PUBLIC cxx_std_17)

# Turtlelib is not a ament target, thus included in this way.
target_link_libraries(leo_ros_utils turtlelib::turtlelib)

# For other packages to find it by cmake
# find package and target_link_libraries(SOME_TARGET leo_utils::leo_ros_utils)
# The following 3 sections are needed.
# However, since this is also a ament cmake package. Simply do
# ament_target_dependencies(SOME_TARGET leo_ros_utils) also works.

# install the include files by copying the whole include directory
# For other ament cmake ros package, this includes srv, launch, config, urdf, etc.
install(DIRECTORY include/leo_ros_utils DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-config.cmake
  NAMESPACE leo_utils::
  DESTINATION lib/cmake/${PROJECT_NAME})

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
