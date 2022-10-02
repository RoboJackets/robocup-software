#include "coach_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto coach = std::make_shared<CoachNode>();
    rclcpp::spin(coach);
}