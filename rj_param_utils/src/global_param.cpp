#include <rj_param_utils/global_param.hpp>
#include <rj_constants/topic_names.hpp>

ROS2GlobalParameterProvider::ROS2GlobalParamProvider(rclcpp::Node* node, 
        const std::string& global_node) : node_{node}, global_node_{global_node} {
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node);

    using rcl_interfaces::msg::SetParametersResult;

    auto on_param_event_callback = 
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
           return UpdateParameters(event->changed_parameters); 
        };

    params_client_sub_ = params_client_->on_parameter_event(on_param_event_callback);
}

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node* global_param_node = std::make_shared<rclcpp::Node>(params::kGlobalParamNodeName);
    global_param_node->declare_parameter("global_bool", false);
    global_param_node->declare_parameter("global_float", 3.14f);
    global_param_node->declare_parameter("global_string", "I am a string.");
    rclcpp::spin(global_param_node);
    rclcpp::shutdown();
    return 0;
}

