#include <rj_utils/logging.hpp>

#include "ball_placement_server.hpp"
#include "global_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto ball_placement_server = std::make_shared<server::BallPlacementServer>();
    start_global_param_provider(ball_placement_server.get(), kGlobalParamServerNode);
    rclcpp::spin(ball_placement_server);
}