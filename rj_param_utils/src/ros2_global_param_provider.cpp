#include <spdlog/spdlog.h>

#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_global_param_provider.hpp>

namespace params {

ROS2GlobalParamProvider::ROS2GlobalParamProvider(rclcpp::Node* node, const std::string& module,
                                                 const std::string& global_node)
    : BaseROS2ParamProvider{module} {
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node);

    // Create a temporary thread to spin the node, just while we're starting up.
    std::atomic_bool is_running{true};
    std::thread spin_thread([node, &is_running]() {
        while (is_running && rclcpp::ok()) {
            rclcpp::spin_some(node->shared_from_this());
        }
    });

    while (!params_client_->wait_for_service(std::chrono::seconds(1))) {
        SPDLOG_WARN("Cannot find global parameter server node with name {}", global_node);
    }
    SPDLOG_INFO("Connected to global parameter server {}", global_node);

    // Make a request to get the parameters for which we locally have param names. This should just
    // be all global parameters.
    auto params_fut = params_client_->get_parameters(GetParamNames());
    params_fut.wait();

    // Filter out all parameters that are unset. We'll get a "new parameter" message when these are
    // eventually set.
    auto params = params_fut.get();
    params.erase(
        std::remove_if(params.begin(), params.end(),
                       [](const auto& p) { return p.get_type() == rclcpp::PARAMETER_NOT_SET; }),
        params.end());

    // Update the parameters with initial values from the server.
    UpdateParameters(params);

    // Kill the ROS thread
    is_running = false;
    spin_thread.join();

    using rcl_interfaces::msg::SetParametersResult;

    auto on_param_event_callback =
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {  // NOLINT
            std::vector<rclcpp::Parameter> updates;
            for (const auto& param_msg : event->changed_parameters) {
                updates.emplace_back(rclcpp::Parameter(param_msg.name, param_msg.value));
            }
            for (const auto& param_msg : event->new_parameters) {
                updates.emplace_back(rclcpp::Parameter(param_msg.name, param_msg.value));
            }
            return UpdateParameters(updates);
        };
    params_client_sub_ = params_client_->on_parameter_event(on_param_event_callback);
}

}  // namespace params