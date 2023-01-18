#include <spdlog/spdlog.h>

#include <rj_utils/logging.hpp>

// TODO: move this main up one level
#include "coach_node.hpp"
#include "strategy/agent/agent_action_client.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto coach = std::make_shared<strategy::CoachNode>(rclcpp::NodeOptions{});
    start_global_param_provider(coach.get(), kGlobalParamServerNode);
    rclcpp::spin(coach);
}
