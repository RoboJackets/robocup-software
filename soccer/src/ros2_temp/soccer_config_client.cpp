#include <ros2_temp/soccer_config_client.h>

namespace ros2_temp {
SoccerConfigClient::SoccerConfigClient(Context* context) : context_{context} {
    config_client_ = std::make_shared<config_client::ConfigClientNode>(
        "soccer_config_client");
    executor_.add_node(config_client_);
}

void SoccerConfigClient::run() {
    config_client_->updateGameSettings(context_->game_settings);
    config_client_->updateFieldDimensions(context_->field_dimensions);

    executor_.spin_some();

    // Check if it is connected, otherwise the below calls are invalid.
    if (!config_client_->connected()) {
        return;
    }

    context_->field_dimensions = config_client_->fieldDimensions();
}
}  // namespace ros2_temp
