#include "soccer_mom.hpp"
#include <rj_common/network.hpp>
#include <rj_utils/logging.hpp>
#include "global_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto soccer_mom = std::make_shared<soccer_mom::SoccerMom>();
    // start_global_param_provider(radio.get(), kGlobalParamServerNode);
    rclcpp::spin(soccer_mom);
}
