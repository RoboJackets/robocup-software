#include "straight_line_test.hpp"
#include "global_params.hpp"
#include "rj_utils/logging.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    rclcpp::executors::MultiThreadedExecutor executor;

    std::vector<rclcpp::Node::SharedPtr> agents;
    for (int i = 0; i < 6; i++) {
        auto agent = std::make_shared<strategy::StraightLineTest>(i);
        start_global_param_provider(agent.get(), kGlobalParamServerNode);
        agents.push_back(agent);
    }
    for (const auto& agent : agents) {
        executor.add_node(agent);
    }
    executor.spin();
}