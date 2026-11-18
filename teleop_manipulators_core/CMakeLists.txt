cmake_minimum_required(VERSION 3.8)
project(teleop_manipulators_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only contract package: types + the ManipulatorInterface plugin base class.
add_library(teleop_manipulators_core INTERFACE)
target_include_directories(teleop_manipulators_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(teleop_manipulators_core INTERFACE Eigen3::Eigen)
target_compile_features(teleop_manipulators_core INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS teleop_manipulators_core
  EXPORT export_teleop_manipulators_core)

ament_export_targets(export_teleop_manipulators_core)
ament_export_dependencies(Eigen3)

ament_package()
