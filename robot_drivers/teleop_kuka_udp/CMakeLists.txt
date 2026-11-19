cmake_minimum_required(VERSION 3.8)
project(teleop_kuka_udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teleop_manipulators_core REQUIRED)

add_library(teleop_kuka_udp SHARED src/kuka_udp_robot.cpp)
target_include_directories(teleop_kuka_udp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_kuka_udp PUBLIC cxx_std_17)
target_link_libraries(teleop_kuka_udp
  teleop_manipulators_core::teleop_manipulators_core)
ament_target_dependencies(teleop_kuka_udp rclcpp pluginlib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS teleop_kuka_udp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(teleop_manipulators_core plugins.xml)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kuka_protocol test/test_kuka_protocol.cpp)
  if(TARGET test_kuka_protocol)
    target_link_libraries(test_kuka_protocol teleop_kuka_udp)
  endif()
endif()

ament_package()
