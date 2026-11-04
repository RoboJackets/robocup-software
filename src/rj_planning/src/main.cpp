#include <rj_param_utils/global_params.hpp>
#include <rj_utils/logging.hpp>

#include "rj_planning/kick_planner_node.hpp"
#include "rj_planning/move_planner_node.hpp"
#include "rj_planning/planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto kick_planner = std::make_shared<planning::KickPlannerNode>();
    auto move_planner = std::make_shared<planning::MovePlannerNode>();
    start_global_param_provider(kick_planner.get(), kGlobalParamServerNode);
    start_global_param_provider(move_planner.get(), kGlobalParamServerNode);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(kick_planner);
    exec.add_node(move_planner);
    exec.spin();
    // rclcpp::shutdown();
}
