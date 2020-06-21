#pragma once

#include <config_client/config_client.h>

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

    [[nodiscard]] inline const GameSettingsMsg& gameSettings() const {
        return config_client_.gameSettings();
    }

    [[nodiscard]] inline const FieldDimensionsMsg& fieldDimensions() const {
        return config_client_.fieldDimensions();
    }

    [[nodiscard]] inline const GameSettingsMsg& gameSettingsThreaded() const {
        return config_client_.gameSettingsThreaded();
    }

    [[nodiscard]] inline const FieldDimensionsMsg& fieldDimensionsThreaded() const {
        return config_client_.fieldDimensionsThreaded();
    }

    [[nodiscard]] inline bool connected() const {
        return config_client_.connected();
    }

    inline void updateGameSettings(const GameSettingsMsg& msg) {
        return config_client_.updateGameSettings(msg);
    }

    inline void updateFieldDimensions(const FieldDimensionsMsg& msg) {
        return config_client_.updateFieldDimensions(msg);
    }

private:
    ConfigClient config_client_;
};
}  // namespace config_client
