#include "internal_referee.hpp"

#include <rj_constants/topic_names.hpp>

namespace referee {

InternalReferee::InternalReferee() : RefereeBase("internal_referee") {
    using rj_msgs::srv::QuickCommands;
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    quick_commands_srv_ = create_service<QuickCommands>(
        referee::topics::kQuickCommandsSrv,
        [this](QuickCommands::Request::SharedPtr
                   request,  // NOLINT(performance-unnecessary-value-param)
               [[maybe_unused]] QuickCommands::Response::SharedPtr
                   response) {  // NOLINT(performance-unnecessary-value-param)
            set_play_state(rj_convert::convert_from_ros(request->command));
        });

    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    game_settings_sub_ = create_subscription<rj_msgs::msg::GameSettings>(
        config_server::topics::kGameSettingsPub, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::GameSettings::SharedPtr msg) {
            set_team_color(msg->request_blue_team);
            override_goalie(msg->request_goalie_id);
            send();
        });
}

}  // namespace referee