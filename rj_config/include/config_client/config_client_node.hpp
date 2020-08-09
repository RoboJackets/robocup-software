#pragma once

#include <config_client/config_client.hpp>

#include <rclcpp/rclcpp.hpp>

namespace config_client {
/**
 * @brief A temporary node to be used by Processor during the ROS2 migration
 * to access the config without being a rclcpp::Node itself.
 *
 * This node simply acts as a wrapper for ConfigClient.
 */
class ConfigClientNode : public rclcpp::Node {
public:
    ConfigClientNode(const std::string& name);

    [[nodiscard]] const GameSettingsMsg& game_settings() const {
        return config_client_.game_settings();
    }

    [[nodiscard]] const FieldDimensionsMsg& field_dimensions() const {
        return config_client_.field_dimensions();
    }

    [[nodiscard]] const GameSettingsMsg& game_settings_threaded() const {
        return config_client_.game_settings_threaded();
    }

    [[nodiscard]] const FieldDimensionsMsg& field_dimensions_threaded() const {
        return config_client_.field_dimensions_threaded();
    }

    [[nodiscard]] bool connected() const { return config_client_.connected(); }

    bool wait_until_connection() const {
        return config_client_.wait_until_connected();
    }

    void update_game_settings(const GameSettingsMsg& msg) {
        return config_client_.update_game_settings(msg);
    }

    void update_field_dimensions(const FieldDimensionsMsg& msg) {
        return config_client_.update_field_dimensions(msg);
    }

private:
    ConfigClient config_client_;
};
}  // namespace config_client
