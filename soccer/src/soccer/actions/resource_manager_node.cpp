#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "resource_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto resource_manager = std::make_shared<actions::ResourceManager>();
    rclcpp::spin(resource_manager);
    rclcpp::shutdown();
}