#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_robocup/msg/field_dimensions.hpp>
#include <rj_robocup/msg/game_settings.hpp>
#include <rj_robocup/srv/set_field_dimensions.hpp>
#include <rj_robocup/srv/set_game_settings.hpp>

namespace config_client {
using GameSettingsMsg = rj_robocup::msg::GameSettings;
using SetGameSettingsSrv = rj_robocup::srv::SetGameSettings;
using SetGameSettingsReq = SetGameSettingsSrv::Request;

using FieldDimensionsMsg = rj_robocup::msg::FieldDimensions;
using SetFieldDimensionsSrv = rj_robocup::srv::SetFieldDimensions;
using SetFieldDimensionsReq = SetFieldDimensionsSrv::Request;

/**
 * \brief Helper util that sets up the proper subscribers for
 * receiving config updates
 */
class ConfigClient {
public:
    ConfigClient(rclcpp::Node* node);

    [[nodiscard]] const GameSettingsMsg& gameSettings() const {
        return game_settings_.value();
    }

    [[nodiscard]] const FieldDimensionsMsg& fieldDimensions() const {
        return field_dimensions_.value();
    }

    [[nodiscard]] bool connected() const {
        const bool has_game_settings = game_settings_.has_value();
        const bool has_field_dimensions = field_dimensions_.has_value();

        const bool have_msg = has_game_settings && has_field_dimensions;

        if (!have_msg) {
            auto& clk = *node_->get_clock();
            const auto throttle_ms = 1000;
            RCLCPP_INFO_STREAM_THROTTLE(
                node_->get_logger(), clk, throttle_ms,
                "[ConfigClient] Waiting on ConfigServer...");
        }
        return have_msg;
    }

    void updateGameSettings(const GameSettingsMsg& msg);
    void updateFieldDimensions(const FieldDimensionsMsg& msg);

private:
    rclcpp::Node* node_;

    rclcpp::Subscription<GameSettingsMsg>::SharedPtr game_settings_sub_;
    rclcpp::Client<SetGameSettingsSrv>::SharedPtr game_settings_client_;
    std::optional<GameSettingsMsg> game_settings_;

    rclcpp::Subscription<FieldDimensionsMsg>::SharedPtr field_dimensions_sub_;
    rclcpp::Client<SetFieldDimensionsSrv>::SharedPtr field_dimensions_client_;
    std::optional<FieldDimensionsMsg> field_dimensions_;
};
}  // namespace config_client
