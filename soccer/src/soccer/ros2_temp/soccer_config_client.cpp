#include <ros2_temp/soccer_config_client.h>

namespace ros2_temp {
SoccerConfigClient::SoccerConfigClient(Context* context) : context_{context} {
    config_client_ = std::make_shared<config_client::ConfigClientNode>(
        "soccer_config_client");
    executor_.add_node(config_client_);

    worker_ = std::thread{&SoccerConfigClient::spin, this};
}

void SoccerConfigClient::spin() { executor_.spin(); }

void SoccerConfigClient::run() {
    // Check if it is connected, otherwise the below calls are invalid.
    if (!config_client_->connected()) {
        return;
    }

    rj_convert::convert_from_ros(config_client_->gameSettingsThreaded(),
                                 &context_->game_settings);
    rj_convert::convert_from_ros(config_client_->fieldDimensionsThreaded(), &context_->field_dimensions);
}
}  // namespace ros2_temp
