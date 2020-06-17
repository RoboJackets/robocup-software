#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/field_dimensions.hpp>
#include <rj_msgs/msg/game_settings.hpp>
#include <rj_msgs/srv/set_field_dimensions.hpp>
#include <rj_msgs/srv/set_game_settings.hpp>

namespace config_server {
using GameSettingsMsg = rj_msgs::msg::GameSettings;
using SetGameSettingsSrv = rj_msgs::srv::SetGameSettings;

using FieldDimensionsMsg = rj_msgs::msg::FieldDimensions;
using SetFieldDimensionsSrv = rj_msgs::srv::SetFieldDimensions;

class ConfigServer : public rclcpp::Node {
public:
    ConfigServer(const rclcpp::NodeOptions& node_options);

private:
    void setGameSettingsCallback(const GameSettingsMsg& game_settings);
    void broadcastGameSettings();

    void setFieldDimensionsCallback(const FieldDimensionsMsg& field_dimensions);
    void broadcastFieldDimensions();

    GameSettingsMsg game_settings_;
    FieldDimensionsMsg field_dimensions_;

    rclcpp::Service<SetGameSettingsSrv>::SharedPtr game_settings_server_;
    rclcpp::Publisher<GameSettingsMsg>::SharedPtr game_settings_publisher_;

    rclcpp::Service<SetFieldDimensionsSrv>::SharedPtr field_dimensions_server_;
    rclcpp::Publisher<FieldDimensionsMsg>::SharedPtr
        field_dimensions_publisher_;
};
}  // namespace config_server
