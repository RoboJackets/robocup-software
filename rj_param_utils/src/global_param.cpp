#include <rj_param_utils/global_param.hpp>

ROS2GlobalParameterProvider::ROS2GlobalParamProvider(rclcpp::Node* node, 
        const std::string& global_node) : node_{node}, global_node_{global_node} {
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node);

    auto on_param_event_callback = 
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
        {
            std::vector<std::string> names;
            for (auto& changed_parameter : event->changed_parameters) {
                names.emplace_back(changed_parameter.name);
                Update(changed_parameter.name, params_client_->get_parameters(names));
                names.erase(names.begin());
            }
        };

    params_client_sub_ = params_client_->on_parameter_event(on_param_event_callback);
}




