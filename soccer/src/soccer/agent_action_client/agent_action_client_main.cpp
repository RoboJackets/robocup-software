#include "agent_action_client.hpp"
#include "global_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto agent_action_client_node = std::make_shared<strategy::AgentActionClient>();
    start_global_param_provider(agent_action_client_node.get(), kGlobalParamServerNode);
    rclcpp::spin(agent_action_client_node);
}
