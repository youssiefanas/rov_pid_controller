#include <rclcpp/rclcpp.hpp>

#include "rov_pid_controller/controller_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_pid_controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
