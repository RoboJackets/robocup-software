#include <rj_utils/logging.hpp>
#include <rj_param_utils/global_params.hpp>

#include "rj_planning/planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto planner = std::make_shared<planning::PlannerNode>();
    start_global_param_provider(planner.get(), kGlobalParamServerNode);
    rclcpp::spin(planner);
}
