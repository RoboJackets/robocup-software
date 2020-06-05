#include <config_server/config_server.h>

#include <rclcpp/rclcpp.hpp>

namespace config_server {
using namespace std::chrono_literals;

ConfigServer::ConfigServer(rclcpp::NodeOptions node_options)
    : Node{"config_server", node_options} {
    const auto latching_qos = rclcpp::QoS(1).transient_local();
    publisher_ =
        create_publisher<GameSettingsMsg>("config/game_settings", latching_qos);

    const auto service_cb =
        [this](const SetGameSettingsSrv::Request::SharedPtr request,
               SetGameSettingsSrv::Response::SharedPtr /*response*/) {
            setGameSettingsCallback(request->game_settings);
        };
    server_ = create_service<SetGameSettingsSrv>("config/set_game_settings",
                                                 service_cb);

    broadcastGameSettings();
}

void ConfigServer::broadcastGameSettings() {
    publisher_->publish(game_settings_);
}

void ConfigServer::setGameSettingsCallback(const GameSettingsMsg& msg) {
    game_settings_ = msg;
    broadcastGameSettings();
}

}  // namespace config_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(config_server::ConfigServer)
