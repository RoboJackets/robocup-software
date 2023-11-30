#include <rj_utils/logging.hpp>

#include "global_params.hpp"
#include "soccermom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto soccermom = std::make_shared<tutorial::SoccerMom>();
    start_global_param_provider(soccermom.get(), kGlobalParamServerNode);
    rclcpp::spin(soccermom);
}
