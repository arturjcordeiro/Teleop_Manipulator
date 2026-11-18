cmake_minimum_required(VERSION 3.8)
project(teleop_ur_rtde)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teleop_manipulators_core REQUIRED)
find_package(ur_rtde REQUIRED)

add_library(teleop_ur_rtde SHARED src/ur_rtde_robot.cpp)
target_include_directories(teleop_ur_rtde PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_ur_rtde PUBLIC cxx_std_17)
target_link_libraries(teleop_ur_rtde
  teleop_manipulators_core::teleop_manipulators_core
  ur_rtde::rtde)
ament_target_dependencies(teleop_ur_rtde rclcpp pluginlib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS teleop_ur_rtde
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(teleop_manipulators_core plugins.xml)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
