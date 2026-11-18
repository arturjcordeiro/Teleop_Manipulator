#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teleop_manipulators_core/manipulator_interface.hpp"

TEST(MockPlugin, LoadsViaPluginlib) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("mock_plugin_test");

  pluginlib::ClassLoader<teleop::ManipulatorInterface> loader(
      "teleop_manipulators_core", "teleop::ManipulatorInterface");
  auto robot = loader.createSharedInstance("mock");
  ASSERT_NE(robot, nullptr);

  teleop::RobotContext ctx{*node, 6, nullptr};
  ASSERT_TRUE(robot->init(ctx));
  EXPECT_EQ(robot->dof(), 6u);
  ASSERT_TRUE(robot->connect());
  ASSERT_TRUE(robot->read().has_value());
  robot->disconnect();
}
