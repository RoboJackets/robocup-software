#include "planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<Planning::PlannerNode>();
    rclcpp::spin(planner);
}