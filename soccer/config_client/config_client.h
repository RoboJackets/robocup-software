#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_robocup/msg/game_settings.hpp>

namespace config_client {
using GameSettingsMsg = rj_robocup::msg::GameSettings;

class ConfigClient : public rclcpp::Node {
public:
    ConfigClient(const std::string& node_name,
                 const rclcpp::NodeOptions& options);

    [[nodiscard]] const GameSettingsMsg& gameSettings() const {
        return game_settings_;
    }

private:
    rclcpp::Subscription<GameSettingsMsg>::SharedPtr subscriber_;
    GameSettingsMsg game_settings_;
};
}  // namespace config_client
