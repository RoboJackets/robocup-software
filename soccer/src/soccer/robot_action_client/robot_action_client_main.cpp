#include "robot_action_client.hpp"
#include "global_params.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    SPDLOG_ERROR("Robot action client is being spun");
    auto robot_action_client_node = std::make_shared<robot_action_client::RobotActionClient>();
    start_global_param_provider(robot_action_client_node.get(), kGlobalParamServerNode);
    rclcpp::spin(robot_action_client_node);
}
