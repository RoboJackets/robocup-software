#include "rj_planning/planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto planner = std::make_shared<planning::PlannerNode>();
    planner->initialize_planners();
    rclcpp::spin(planner);
}
