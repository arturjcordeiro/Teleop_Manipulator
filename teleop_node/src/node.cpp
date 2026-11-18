#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "teleop_manipulators/teleop_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);

  rclcpp::spin(std::make_shared<teleop::TeleopManipulator>(node_options));
  rclcpp::shutdown();

  return 0;
}
