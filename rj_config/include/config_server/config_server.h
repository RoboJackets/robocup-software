#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/field_dimensions.hpp>
#include <rj_msgs/msg/game_settings.hpp>
#include <rj_msgs/msg/game_state.hpp>
#include <rj_msgs/srv/set_field_dimensions.hpp>
#include <rj_msgs/srv/set_game_settings.hpp>
#include <rj_msgs/srv/set_game_state.hpp>

namespace config_server {
using GameSettingsMsg = rj_msgs::msg::GameSettings;
using SetGameSettingsSrv = rj_msgs::srv::SetGameSettings;
using SetGameSettingsSrvReqPtr = SetGameSettingsSrv::Request::SharedPtr;
using SetGameSettingsSrvRespPtr = SetGameSettingsSrv::Response::SharedPtr;

using FieldDimensionsMsg = rj_msgs::msg::FieldDimensions;
using SetFieldDimensionsSrv = rj_msgs::srv::SetFieldDimensions;
using SetFieldDimensionsSrvReqPtr = SetFieldDimensionsSrv::Request::SharedPtr;
using SetFieldDimensionsSrvRespPtr = SetFieldDimensionsSrv::Response::SharedPtr;

using GameStateMsg = rj_msgs::msg::GameState;
using SetGameStateSrv = rj_msgs::srv::SetGameState;
using SetGameStateSrvReqPtr = SetGameStateSrv::Request::SharedPtr;
using SetGameStateSrvRespPtr = SetGameStateSrv::Response::SharedPtr;

/**
 * @brief This node acts as a configuration server, serving all the configs
 * by publishing them with a "Transient local" durability. The node exposes
 * a service call to allow clients to update the configuration, which triggers
 * a republish.
 */
class ConfigServer : public rclcpp::Node {
public:
    /**
     * @brief Constructor.
     * @param node_options
     */
    ConfigServer(const rclcpp::NodeOptions& node_options);

private:
    /**
     * @brief Callback for the GameSettings service. Updates game_settings_.
     * @param msg
     */
    void setGameSettingsCallback(const GameSettingsMsg& msg);

    /**
     * @brief Publishes game_settings_.
     */
    void broadcastGameSettings();

    /**
     * @brief Callback for the FieldDimensions service.
     * Updates field_dimensions_.
     * @param msg
     */
    void setFieldDimensionsCallback(const FieldDimensionsMsg& msg);

    /**
     * @brief Publishes field_dimensions_.
     */
    void broadcastFieldDimensions();

    /**
     * @brief Callback for the GameState service.
     * Updates game_state_.
     * @param msg
     */
    void setGameStateCallback(const GameStateMsg& msg);

    /**
     * @brief Publishes game_state_.
     */
    void broadcastGameState();

    GameSettingsMsg game_settings_;
    FieldDimensionsMsg field_dimensions_;
    GameStateMsg game_state_;

    rclcpp::Service<SetGameSettingsSrv>::SharedPtr game_settings_server_;
    rclcpp::Publisher<GameSettingsMsg>::SharedPtr game_settings_publisher_;

    rclcpp::Service<SetFieldDimensionsSrv>::SharedPtr field_dimensions_server_;
    rclcpp::Publisher<FieldDimensionsMsg>::SharedPtr
        field_dimensions_publisher_;

    rclcpp::Service<SetGameStateSrv>::SharedPtr game_state_server_;
    rclcpp::Publisher<GameStateMsg>::SharedPtr game_state_publisher_;
};
}  // namespace config_server
