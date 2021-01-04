#include "planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<planning::PlannerNode>();
    rclcpp::spin(planner);
}