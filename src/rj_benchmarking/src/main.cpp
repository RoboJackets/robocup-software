#include "rj_benchmarking/registry.hpp"
#include "rj_benchmarking/registry_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Registry>());
    rclcpp::shutdown();
    return 0;
}
