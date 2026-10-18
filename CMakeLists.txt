cmake_minimum_required(VERSION 3.8)
project(teleop_manipulators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ibot_common REQUIRED)

# -----------------------------------------------------------------------------
# Component library: the node implementation. It can be loaded into an
# rclcpp_components container together with the robot driver, which keeps the
# teleop command path intra-process and low latency.
# -----------------------------------------------------------------------------
add_library(teleop_manipulator_component SHARED
  src/teleop_node.cpp)

target_include_directories(teleop_manipulator_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(teleop_manipulator_component PUBLIC cxx_std_17)

ament_target_dependencies(
  teleop_manipulator_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "ibot_common"
)

rclcpp_components_register_node(teleop_manipulator_component
  PLUGIN "teleop::TeleopManipulator"
  EXECUTABLE teleop_manipulator_component_node)

# -----------------------------------------------------------------------------
# Standalone executable: thin entry point for development and debugging.
# -----------------------------------------------------------------------------
add_executable(teleop_manipulators_node
  src/node.cpp)

target_include_directories(teleop_manipulators_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(teleop_manipulators_node PUBLIC cxx_std_17)

target_link_libraries(teleop_manipulators_node
  teleop_manipulator_component)

ament_target_dependencies(
  teleop_manipulators_node
  "rclcpp"
)

# Export the headers so the node can be reused by other packages.
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS
  teleop_manipulator_component
  teleop_manipulators_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
