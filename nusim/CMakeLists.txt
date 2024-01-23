cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}_IDL "srv/Teleport.srv"
  LIBRARY_NAME ${PROJECT_NAME} # This is a necessary line. And it MUST be PROJECT_NAME !! 
  
  # DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in
  # this case geometry_msgs for Sphere.msg
  )
  
# LIBRARY_NAME ${PROJECT_NAME} # This is a necessary line, Otherweise will get This error: 
# 
# [nusim-1] terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
# [nusim-1]   what():  could not create service: Type support not from this implementation. Got:
# [nusim-1]     Handle's typesupport identifier (rosidl_typesupport_cpp) is not supported by this library, at ./src/type_support_dispatch.hpp:111
# [nusim-1]     Could not load library libnusim__rosidl_typesupport_fastrtps_cpp.so: dlopen error: libnusim__rosidl_typesupport_fastrtps_cpp.so: cannot open shared object file: No such file or directory, at ./src/shared_library.c:99, at ./src/type_support_dispatch.hpp:76
#
# Which in build folder: libnusim_srv__rosidl_typesupport_fastrtps_cpp.so exsits, but the 
# Executable is looking for libnusim__rosidl_typesupport_fastrtps_cpp.so


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_IDL
                              "rosidl_typesupport_cpp")

add_executable(nusim src/nusim.cpp)

# https://robotics.stackexchange.com/questions/23171/creating-a-custom-msg-in-ros2
ament_target_dependencies(nusim rclcpp std_msgs std_srvs tf2 tf2_ros)
target_link_libraries(nusim ${cpp_typesupport_target})
install(TARGETS nusim DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
                  srv # Since this is built, I shouldn't need to install it?
                  config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
