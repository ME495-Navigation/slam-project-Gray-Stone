cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_idl
  "srv/Teleport.srv"
  # DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_idl "rosidl_typesupport_cpp")


add_executable(nusim src/nusim.cpp)


# https://robotics.stackexchange.com/questions/23171/creating-a-custom-msg-in-ros2
ament_target_dependencies(nusim rclcpp std_msgs std_srvs)
target_link_libraries(nusim ${cpp_typesupport_target})
install(TARGETS
  nusim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  # srv # Since this is built, I shouldn't need to install it?
  config
  DESTINATION share/${PROJECT_NAME}/
)


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
