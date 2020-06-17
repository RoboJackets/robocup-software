#pragma once

#include <config_client/config_client.h>

#include <rclcpp/rclcpp.hpp>

namespace config_client {
class ConfigClientNode : public rclcpp::Node {
public:
    ConfigClientNode(const std::string& name);

    [[nodiscard]] inline const GameSettingsMsg& gameSettings() const {
        return config_client_.gameSettings();
    }

    [[nodiscard]] inline const FieldDimensionsMsg& fieldDimensions() const {
        return config_client_.fieldDimensions();
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
