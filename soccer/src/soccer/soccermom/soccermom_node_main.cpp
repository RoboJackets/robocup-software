#include <rj_common/network.hpp>
#include <rj_utils/logging.hpp>

#include "global_params.hpp"
#include "soccermom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto soccermom = std::make_shared<Soccermom>();
    rclcpp::spin(soccermom);
}
