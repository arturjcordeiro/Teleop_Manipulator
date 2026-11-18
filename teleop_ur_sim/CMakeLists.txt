cmake_minimum_required(VERSION 3.8)
project(teleop_ur_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teleop_manipulators_core REQUIRED)

add_library(teleop_ur_sim SHARED src/ur_sim_robot.cpp)
target_include_directories(teleop_ur_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_ur_sim PUBLIC cxx_std_17)
target_link_libraries(teleop_ur_sim
  teleop_manipulators_core::teleop_manipulators_core)
ament_target_dependencies(teleop_ur_sim rclcpp sensor_msgs std_msgs pluginlib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS teleop_ur_sim
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(teleop_manipulators_core plugins.xml)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
