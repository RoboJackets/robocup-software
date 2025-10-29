#include <rclcpp/rclcpp.hpp>

#include <rj_benchmarking/benchmarking.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<benchmarking::Benchmarking>());
    rclcpp::shutdown();
    return 0;
}