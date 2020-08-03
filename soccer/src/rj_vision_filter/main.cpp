#include <rclcpp/rclcpp.hpp>
#include <rj_vision_filter/VisionFilter.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor{};

    const auto node =
        std::make_shared<vision_filter::VisionFilter>(rclcpp::NodeOptions{});
    executor.add_node(node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
