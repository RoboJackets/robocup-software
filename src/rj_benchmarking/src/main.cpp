#include <rclcpp/rclcpp.hpp>

#include "rj_benchmarking/benchmarking.hpp"
#include "rj_benchmarking/registry.hpp"
#include "rj_benchmarking/timer.hpp"


Registry* Registry::instance = nullptr;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Benchmarking>());
    {
        Timer t("lol", 1);
    }
    rclcpp::shutdown();
    return 0;
}
