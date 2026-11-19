cmake_minimum_required(VERSION 3.8)
project(teleop_mock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teleop_manipulators_core REQUIRED)

add_library(teleop_mock SHARED src/mock_robot.cpp)
target_include_directories(teleop_mock PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_mock PUBLIC cxx_std_17)
target_link_libraries(teleop_mock
  teleop_manipulators_core::teleop_manipulators_core)
ament_target_dependencies(teleop_mock rclcpp pluginlib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS teleop_mock
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(teleop_manipulators_core plugins.xml)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  if(TARGET test_plugin_load)
    target_link_libraries(test_plugin_load
      teleop_mock
      teleop_manipulators_core::teleop_manipulators_core)
    ament_target_dependencies(test_plugin_load rclcpp pluginlib)
  endif()
endif()

ament_package()
