#include <rclcpp/rclcpp.hpp>

#include <rj_benchmarking/registry.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<benchmarking::Registry>());
    rclcpp::shutdown();
    return 0;
}