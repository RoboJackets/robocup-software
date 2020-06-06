#include "config_server.h"

#include <rclcpp/rclcpp.hpp>

using config_server::ConfigServer;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConfigServer>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
