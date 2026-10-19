#include <rj_param_utils/global_params.hpp>

#include "rj_planning/planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto planner = std::make_shared<planning::PlannerNode>();
    planner->initialize_planners();
    start_global_param_provider(planner.get(), kGlobalParamServerNode);
    rclcpp::spin(planner);
}
