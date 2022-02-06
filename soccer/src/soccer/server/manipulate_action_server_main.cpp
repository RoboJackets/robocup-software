#include <rj_utils/logging.hpp>

#include "global_params.hpp"
#include "manipulate_action_server.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto manipulate_action_server = std::make_shared<server::ManipulateActionServer>();
    start_global_param_provider(manipulate_action_server.get(), kGlobalParamServerNode);
    rclcpp::spin(manipulate_action_server);
}
