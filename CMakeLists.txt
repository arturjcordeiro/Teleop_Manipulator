cmake_minimum_required(VERSION 3.8)
project(teleop_manipulators)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ibot_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ur_rtde QUIET)

# -----------------------------------------------------------------------------
# Component library: the node implementation. It can be loaded into an
# rclcpp_components container together with the robot driver, which keeps the
# teleop command path intra-process and low latency.
# -----------------------------------------------------------------------------
add_library(teleop_core STATIC
  src/core/pose_mapper.cpp
  src/core/diff_ik.cpp
  src/core/safety.cpp
)
target_include_directories(teleop_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(teleop_core PUBLIC cxx_std_17)
target_link_libraries(teleop_core
  PUBLIC  Eigen3::Eigen
  PRIVATE pinocchio::pinocchio   # hidden behind pimpl in diff_ik
)


add_library(teleop_backends STATIC
  src/backends/mock_robot.cpp
  src/backends/kuka_udp_robot.cpp
  src/backends/robot_factory.cpp
)
target_compile_features(teleop_backends PUBLIC cxx_std_17)
target_link_libraries(teleop_backends PUBLIC teleop_core)



if(ur_rtde_FOUND)
  message(STATUS "ur_rtde found: UR backend enabled")
  target_sources(teleop_backends PRIVATE src/backends/ur_rtde_robot.cpp)
  target_link_libraries(teleop_backends PRIVATE ur_rtde::rtde)
  target_compile_definitions(teleop_backends PRIVATE TELEOP_HAS_UR)
else()
  message(STATUS "ur_rtde not found: UR backend disabled")
endif()

add_library(teleop_manipulator_component SHARED
  src/teleop_node.cpp)
target_include_directories(teleop_manipulator_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teleop_manipulator_component PUBLIC cxx_std_17)
target_link_libraries(teleop_manipulator_component teleop_backends)

ament_target_dependencies(
  teleop_manipulator_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "std_srvs"
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
  teleop_core
  teleop_backends
  teleop_manipulator_component
  teleop_manipulators_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_srvs
  ibot_common
  Eigen3
  pinocchio
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gtest(test_pose_mapper test/test_pose_mapper.cpp)
  if(TARGET test_pose_mapper)
    target_link_libraries(test_pose_mapper teleop_core)
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
