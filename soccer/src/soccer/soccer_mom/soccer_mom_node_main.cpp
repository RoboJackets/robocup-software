//#include <rj_common/network.hpp>
#include <rj_utils/logging.hpp>

//#include "global_params.hpp"
//#include "sim_radio.hpp"

#include "soccer_mom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto soccer_mom = std::make_shared<SoccerMom>();
//    start_global_param_provider(radio.get(), kGlobalParamServerNode);
    rclcpp::spin(soccer_mom);
}
