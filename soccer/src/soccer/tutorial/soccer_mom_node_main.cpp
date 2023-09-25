#include <rj_common/network.hpp>
#include <rj_utils/logging.hpp>

#include "global_params.hpp"
#include "soccer_mom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto soccer_mom = std::make_shared<tutorial::SoccerMom>();
    start_global_param_provider(soccer_mom.get(), kGlobalParamServerNode);
    rclcpp::spin(soccer_mom);
}
