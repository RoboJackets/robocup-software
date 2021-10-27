#include <rj_utils/logging.hpp>

#include "global_params.hpp"
#include "move_action_server.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");
    
    auto move_action_server = std::make_shared<server::MoveActionServer>();
    start_global_param_provider(move_action_server.get(), kGlobalParamServerNode);
    rclcpp::spin(move_action_server);
}
