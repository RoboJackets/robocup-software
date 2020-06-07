#include <config_server/config_server.h>

#include <rclcpp/rclcpp.hpp>

namespace config_server {
ConfigServer::ConfigServer(rclcpp::NodeOptions node_options)
    : Node{"config_server", node_options} {
  const auto latching_qos = rclcpp::QoS(1).transient_local();

  // Game Settings
  game_settings_publisher_ =
      create_publisher<GameSettingsMsg>("config/game_settings", latching_qos);

  const auto game_settings_cb =
      [this](const SetGameSettingsSrv::Request::SharedPtr request,
             SetGameSettingsSrv::Response::SharedPtr /*response*/) {
        setGameSettingsCallback(request->game_settings);
      };
  game_settings_server_ = create_service<SetGameSettingsSrv>(
      "config/set_game_settings", game_settings_cb);

  // Field Dimensions
  field_dimensions_publisher_ = create_publisher<FieldDimensionsMsg>(
      "config/field_dimensions", latching_qos);

  const auto field_dimensions_cb =
      [this](const SetFieldDimensionsSrv::Request::SharedPtr request,
             SetFieldDimensionsSrv::Response::SharedPtr /*response*/) {
        setFieldDimensionsCallback(request->field_dimensions);
      };
  field_dimensions_server_ = create_service<SetFieldDimensionsSrv>(
      "config/set_field_dimensions", field_dimensions_cb);

  broadcastFieldDimensions();
}

void ConfigServer::broadcastGameSettings() {
  game_settings_publisher_->publish(game_settings_);
}

void ConfigServer::setGameSettingsCallback(const GameSettingsMsg& msg) {
  game_settings_ = msg;
  broadcastGameSettings();
}

void ConfigServer::broadcastFieldDimensions() {
  field_dimensions_publisher_->publish(field_dimensions_);
}

void ConfigServer::setFieldDimensionsCallback(const FieldDimensionsMsg& msg) {
  field_dimensions_ = msg;
  broadcastFieldDimensions();
}

}  // namespace config_server
