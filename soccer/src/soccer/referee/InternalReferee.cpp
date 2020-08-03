#include "InternalReferee.hpp"

#include <rj_constants/topic_names.hpp>

namespace referee {

InternalReferee::InternalReferee() : RefereeBase("internal_referee") {
    using rj_msgs::srv::QuickCommands;
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    _quick_commands_srv = create_service<QuickCommands>(
        referee::topics::kQuickCommandsSrv,
        [this](QuickCommands::Request::SharedPtr
                   request,  // NOLINT(performance-unnecessary-value-param)
               [[maybe_unused]] QuickCommands::Response::SharedPtr
                   response) {  // NOLINT(performance-unnecessary-value-param)
            switch (request->state) {
                case QuickCommands::Request::COMMAND_HALT:
                    halt();
                    break;
                case QuickCommands::Request::COMMAND_STOP:
                    stop();
                    break;
                case QuickCommands::Request::COMMAND_READY:
                    ready();
                    break;
                case QuickCommands::Request::COMMAND_PLAY:
                    play();
                    break;
                default:
                    throw std::runtime_error("Unknown quick command " +
                                             std::to_string(request->state));
            }
            send();
        });

    using rj_msgs::srv::QuickRestart;
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    _quick_restart_srv = create_service<QuickRestart>(
        referee::topics::kQuickRestartSrv,
        [this](QuickRestart::Request::SharedPtr
                   request,  // NOLINT(performance-unnecessary-value-param)
               [[maybe_unused]] QuickRestart::Response::SharedPtr
                   response) {  // NOLINT(performance-unnecessary-value-param)
            switch (request->restart) {
                case QuickRestart::Request::RESTART_DIRECT:
                    restart(GameState::Restart::Direct, request->blue_team);
                    break;
                case QuickRestart::Request::RESTART_INDIRECT:
                    restart(GameState::Restart::Indirect, request->blue_team);
                    break;
                case QuickRestart::Request::RESTART_KICKOFF:
                    restart(GameState::Restart::Kickoff, request->blue_team);
                    break;
                default:
                    throw std::runtime_error("Unknown quick restart " +
                                             std::to_string(request->restart));
            }
            send();
        });

    _game_settings_sub = create_subscription<rj_msgs::msg::GameSettings>(
        config_server::topics::kGameSettingsPub,
        rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::GameSettings::SharedPtr
                   msg) {  // NOLINT(performance-unnecessary-value-param)
            set_team_color(msg->request_blue_team);
            set_goalie(msg->request_goalie_id);
        });
}

}  // namespace referee