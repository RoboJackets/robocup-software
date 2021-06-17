#include <rclcpp/rclcpp.hpp>

#include "vision/vision_filter.hpp"

#include "global_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor{};

    const auto node = std::make_shared<vision_filter::VisionFilter>(rclcpp::NodeOptions{});
    start_global_param_provider(node.get(), kGlobalParamServerNode);
    executor.add_node(node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
