#include "agent_action_client.hpp"
#include "global_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // spin up one action client for each robot
    // (must be added to a vector so shared_ptrs aren't deleted when they go out of scope)
    std::vector<rclcpp::Node::SharedPtr> nodes;
    for (int i = 0; i < 6;
        i++) {  // TODO (Kevin): make this kNumShells and brick the non-used shells
        SPDLOG_INFO("HERE 1");
        auto agent = std::make_shared<strategy::AgentActionClient>(i);
        start_global_param_provider(agent.get()->node().get(), kGlobalParamServerNode);
        nodes.push_back(agent.get()->node());
        SPDLOG_INFO("HERE 2");
    }
    for (const auto& node : nodes) {
        executor.add_node(node);
    }
    executor.spin();
}
