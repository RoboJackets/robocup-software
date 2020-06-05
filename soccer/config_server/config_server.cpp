#include <config_server/config_server.h>
#include <rclcpp/rclcpp.hpp>

namespace config_server {
using namespace std::chrono_literals;

ConfigServer::ConfigServer(rclcpp::NodeOptions node_options)
    : Node{"config_server", node_options} {
    publisher_ = create_publisher<GameSettingsMsg>("config/game_settings", 10);
    server_ = create_service<SetGameSettingsSrv>(
        "config/set_game_settings",
        [this](const SetGameSettingsSrv::Request::SharedPtr request, SetGameSettingsSrv::Response::SharedPtr) {
            setGameSettingsCallback(request->game_settings);
        });
}

void ConfigServer::broadcastGameSettings() {
    publisher_->publish(game_settings_);
}

void ConfigServer::setGameSettingsCallback(const GameSettingsMsg& msg) {
    game_settings_ = msg;
    RCLCPP_INFO_STREAM(get_logger(), "callback!");
}

}  // namespace config_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(config_server::ConfigServer)
