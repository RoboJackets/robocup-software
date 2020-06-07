#include <config_client/config_client.h>

namespace config_client {
ConfigClient::ConfigClient(rclcpp::Node* node) : node_{node} {
  const auto latching_qos = rclcpp::QoS(1).transient_local();

  const auto game_settings_cb = [this](GameSettingsMsg::UniquePtr msg) {
    game_settings_ = *msg;
  };
  game_settings_sub_ = node->create_subscription<GameSettingsMsg>(
      "/config/game_settings", latching_qos, game_settings_cb);

  const auto field_dimensions_cb = [this](FieldDimensionsMsg::UniquePtr msg) {
    field_dimensions_ = *msg;
  };
  field_dimensions_sub_ = node->create_subscription<FieldDimensionsMsg>(
      "/config/field_dimensions", latching_qos, field_dimensions_cb);
}

void ConfigClient::updateGameSettings(GameSettingsMsg msg) {
  SetGameSettingsReq::SharedPtr request =
      std::make_shared<SetGameSettingsReq>();
  request->game_settings = msg;

  game_settings_client_->async_send_request(request);
}

void ConfigClient::updateFieldDimensions(FieldDimensionsMsg msg) {
  SetFieldDimensionsReq::SharedPtr request =
      std::make_shared<SetFieldDimensionsReq>();
  request->field_dimensions = msg;

  field_dimensions_client_->async_send_request(request);
}
}  // namespace config_client
