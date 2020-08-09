#include <rclcpp/rclcpp.hpp>

#include <config_server/config_server.hpp>

using config_server::ConfigServer;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConfigServer>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
