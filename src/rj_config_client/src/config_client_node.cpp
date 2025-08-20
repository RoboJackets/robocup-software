#include <rclcpp/rclcpp.hpp>

#include <rj_config_client/config_client_node.hpp>

namespace config_client {
ConfigClientNode::ConfigClientNode(const std::string& name) : Node{name}, config_client_(this) {}
}  // namespace config_client

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<config_client::ConfigClientNode>("config_client"));
    rclcpp::shutdown();
    return 0;
}