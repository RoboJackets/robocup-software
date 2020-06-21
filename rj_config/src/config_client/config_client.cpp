#include <config_client/config_client.h>
#include <rj_constants/topic_names.hpp>

#include <rj_utils/logging.hpp>

namespace config_client {
ConfigClient::ConfigClient(rclcpp::Node* node) : node_{node} {
    const auto latching_qos = rclcpp::QoS(1).transient_local();

    const auto game_settings_cb = [this](GameSettingsMsg::UniquePtr msg) {
        game_settings_ = *msg;
    };
    game_settings_sub_ = node->create_subscription<GameSettingsMsg>(
        config_server::topics::kGameSettingsPub, latching_qos, game_settings_cb);
    game_settings_client_ =
        node->create_client<SetGameSettingsSrv>(config_server::topics::kGameSettingsSrv);

    const auto field_dimensions_cb = [this](FieldDimensionsMsg::UniquePtr msg) {
        field_dimensions_ = *msg;
    };
    field_dimensions_sub_ = node->create_subscription<FieldDimensionsMsg>(
        config_server::topics::kFieldDimensionsPub, latching_qos, field_dimensions_cb);
    field_dimensions_client_ = node->create_client<SetFieldDimensionsSrv>(
        config_server::topics::kFieldDimensionsSrv);
}

bool ConfigClient::connected() const {
    const bool has_game_settings = game_settings_.has_value();
    const bool has_field_dimensions = field_dimensions_.has_value();

    const bool have_msg = has_game_settings && has_field_dimensions;

    if (!have_msg) {
        auto& clk = *node_->get_clock();
        const auto throttle_ms = 1000;
        RJ_INFO_STREAM_THROTTLE(node_->get_logger(), clk, throttle_ms,
                                "[ConfigClient] Waiting on ConfigServer...");
    }
    return have_msg;
}

void ConfigClient::updateGameSettings(const GameSettingsMsg& msg) {
    SetGameSettingsReq::SharedPtr request =
        std::make_shared<SetGameSettingsReq>();
    request->game_settings = msg;

    game_settings_client_->async_send_request(request);
}

void ConfigClient::updateFieldDimensions(const FieldDimensionsMsg& msg) {
    SetFieldDimensionsReq::SharedPtr request =
        std::make_shared<SetFieldDimensionsReq>();
    request->field_dimensions = msg;

    field_dimensions_client_->async_send_request(request);
}
}  // namespace config_client
