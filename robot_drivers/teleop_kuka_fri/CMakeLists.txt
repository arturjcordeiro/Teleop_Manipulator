cmake_minimum_required(VERSION 3.8)
project(teleop_kuka_fri)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teleop_manipulators_core REQUIRED)
find_package(FRIClient REQUIRED)

add_library(teleop_kuka_fri SHARED src/kuka_fri_robot.cpp)
target_include_directories(teleop_kuka_fri PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_kuka_fri PUBLIC cxx_std_17)
target_link_libraries(teleop_kuka_fri
  teleop_manipulators_core::teleop_manipulators_core
  FRIClient::FRIClient)
ament_target_dependencies(teleop_kuka_fri rclcpp pluginlib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS teleop_kuka_fri
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(teleop_manipulators_core plugins.xml)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
