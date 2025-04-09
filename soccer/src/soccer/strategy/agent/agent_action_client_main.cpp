#include "agent_action_client.hpp"
#include "global_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    rclcpp::executors::MultiThreadedExecutor executor;

    // spin up one action client for each robot
    // (must be added to a vector so shared_ptrs aren't deleted when they go out of scope)
    std::list<strategy::AgentActionClient> agents; // Linked List because no move operator in AAC
    for (int i = 0; i < 6; ++i) {  // TODO (Kevin): make this kNumShells and brick the non-used shells
        agents.emplace_back(i);
        auto& agent = agents.back();
        start_global_param_provider(agent.node().get(), kGlobalParamServerNode);
        executor.add_node(agent.node());
    }
    executor.spin();
}
