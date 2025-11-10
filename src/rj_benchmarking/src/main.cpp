#include <rclcpp/rclcpp.hpp>

// #include "rj_benchmarking/benchmarking.hpp"
#include "rj_benchmarking/registry.hpp"
// #include "rj_benchmarking/timer.hpp"
#include "rj_benchmarking/registry_publisher.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Registry>());
    // {
    //     Timer t("lol", 1);
    // }
    rclcpp::shutdown();
    return 0;
}
