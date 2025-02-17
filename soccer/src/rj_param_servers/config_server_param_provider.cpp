#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

class ConfigServerParamProvider : public rclcpp::Node
{
public:
    ConfigServerParamProvider() : Node("config_server_param_provider",
                                rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)) {

        declare_parameter("be_goofy", 1.5);
    }
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConfigServerParamProvider>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
