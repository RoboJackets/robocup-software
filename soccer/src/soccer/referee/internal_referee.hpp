#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/srv/quick_commands.hpp>
#include <rj_msgs/srv/quick_restart.hpp>

#include "referee_base.hpp"
#include "config_client/config_client.h"

namespace referee {

/**
 * @brief Responsible for handling "quick actions" (UI start/stop/restart
 * buttons) for fake matches.
 *
 * Exposes quick commands and restarts on @ref
 * referee::topics::kQuickCommandsSrv and @ref
 * referee::topics::kQuickRestartsSrv.
 */
class InternalReferee : public RefereeBase {
public:
    InternalReferee();

private:
    // TODO(#1559): Add another service for ball placement
    rclcpp::Service<rj_msgs::srv::QuickCommands>::SharedPtr quick_commands_srv_;
    rclcpp::Service<rj_msgs::srv::QuickRestart>::SharedPtr quick_restart_srv_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr
        game_settings_sub_;
};

}  // namespace referee