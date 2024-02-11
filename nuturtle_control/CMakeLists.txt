cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(turtlelib REQUIRED)
find_package(leo_ros_utils REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Doxygen)
option(BUILD_DOCS "Build the documentation" OFF)

# IDL stuff
rosidl_generate_interfaces(
  ${PROJECT_NAME}_IDL
  "srv/InitPose.srv"
  "srv/Control.srv"
  LIBRARY_NAME
  ${PROJECT_NAME} # This is a necessary line. And it MUST be PROJECT_NAME !!

  # DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in
  # this case geometry_msgs for Sphere.msg
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_IDL
  "rosidl_typesupport_cpp")

# Executables
add_executable(turtle_control src/turtle_control.cpp)
ament_target_dependencies(
  turtle_control
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nuturtlebot_msgs
  leo_ros_utils)
target_link_libraries(turtle_control turtlelib::turtlelib)

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(
  odometry
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nuturtlebot_msgs
  nav_msgs
  tf2
  leo_ros_utils
  tf2_ros)
target_link_libraries(odometry turtlelib::turtlelib
  ${cpp_typesupport_target})
install(TARGETS turtle_control odometry DESTINATION lib/${PROJECT_NAME})

add_executable(circle src/circle.cpp)
ament_target_dependencies(
  circle
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nuturtlebot_msgs
  nav_msgs
  tf2
  leo_ros_utils
  tf2_ros
  std_srvs)
target_link_libraries(circle turtlelib::turtlelib
  ${cpp_typesupport_target})
install(TARGETS turtle_control odometry circle DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include(CTest)
  find_package(catch_ros2 REQUIRED)

  # ##########################
  # # INTEGRATION TEST NODE ##
  # ##########################
  add_executable(turtle_control_test test/turtle_control_test.cpp)
  target_link_libraries(turtle_control_test
    catch_ros2::catch_ros2_with_node_main)

  ament_target_dependencies(turtle_control_test
    rclcpp
    std_srvs
    geometry_msgs
    sensor_msgs
    nuturtlebot_msgs
    leo_ros_utils
  )

  install(TARGETS
    turtle_control_test
    DESTINATION lib/${PROJECT_NAME}
  )

  # ################
  # # LAUNCH FILE ##
  # ################
  # This is the launch file that will be used to run the integration test
  install(FILES
    test/turtle_control_test.launch.xml # example in XML
    DESTINATION share/${PROJECT_NAME}
  )

  # ##################
  # # Register TEST ##
  # ##################
  catch_ros2_add_integration_test(ExampleIntegration_TestXML
    LAUNCH_FILE turtle_control_test.launch.xml # example in XML
  )

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
