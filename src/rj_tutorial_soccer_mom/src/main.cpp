#include <rclcpp/rclcpp.hpp>

#include "rj_tutorial_soccer_mom/soccer_mom_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tutorial::SoccerMomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
