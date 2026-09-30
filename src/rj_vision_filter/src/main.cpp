#include <rclcpp/rclcpp.hpp>

#include <rj_vision_filter/vision_filter.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor{};
    auto options = rclcpp::NodeOptions{}
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true);
    const auto node = std::make_shared<vision_filter::VisionFilter>(options);
    executor.add_node(node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
