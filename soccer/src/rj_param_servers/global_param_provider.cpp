#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

class GlobalParamProvider : public rclcpp::Node
{
public:
    GlobalParamProvider() : Node("global_param_provider",
                                rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)) {}
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalParamProvider>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


