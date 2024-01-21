 
# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(turtlelib)

find_package(Doxygen)


option(BUILD_DOCS "Build the documentation" OFF)

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(turtlelib src/geometry2d.cpp src/se2d.cpp src/svg.cpp)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(turtlelib PUBLIC cxx_std_17)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
# add_executable(Name src/Name_main.cpp)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
# target_link_libraries(Name turtlelib)

# install the include files by copying the whole include directory
install(DIRECTORY include/turtlelib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS turtlelib EXPORT project_name-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT project_name-targets
        FILE project_name-config.cmake
        NAMESPACE project_name::
        DESTINATION lib/cmake/${PROJECT_NAME})



include(CTest)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    add_executable(test_geometry2d tests/test_geometry2d.cpp)
    target_link_libraries(test_geometry2d Catch2::Catch2WithMain turtlelib)
    add_executable(test_se2d tests/test_se2d.cpp)
    target_link_libraries(test_se2d Catch2::Catch2WithMain turtlelib)
    add_executable(test_svg tests/test_svg.cpp)
    target_link_libraries(test_svg Catch2::Catch2WithMain turtlelib)
    add_executable(manual_test_se2d_svg tests/manual_test_se2d_svg.cpp)
    target_link_libraries(manual_test_se2d_svg Catch2::Catch2WithMain turtlelib)

    # register the test with CTest, telling it what executable to run
    add_test(NAME geometry2d_test COMMAND test_geometry2d)
    add_test(NAME se2d_test COMMAND test_se2d)
endif()


if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()
