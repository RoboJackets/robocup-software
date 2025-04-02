#include "agent_action_client.hpp"
#include "global_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // spin up one action client for each robot
    // (must be added to a vector so shared_ptrs aren't deleted when they go out of scope)
    strategy::AgentActionClient agents[6] = {strategy::AgentActionClient(0), strategy::AgentActionClient(1), strategy::AgentActionClient(2), strategy::AgentActionClient(3), strategy::AgentActionClient(4), strategy::AgentActionClient(5)};
    for (const auto& agent : agents) {  // TODO (Kevin): make this kNumShells and brick the non-used shells
        start_global_param_provider(agent.node().get(), kGlobalParamServerNode);
        executor.add_node(agent.node());
    }
    executor.spin();
}
