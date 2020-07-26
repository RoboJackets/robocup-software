#include "InternalReferee.hpp"

#include <rj_constants/topic_names.hpp>

namespace referee {

InternalReferee::InternalReferee() : RefereeBase("internal_referee") {
    using rj_msgs::srv::QuickCommands;
    _quick_commands_srv = create_service<QuickCommands>(
        referee::topics::kQuickCommandsSrv,
        [this](QuickCommands::Request::SharedPtr request,
               QuickCommands::Response::SharedPtr /* response */) {
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
    _quick_restart_srv = create_service<QuickRestart>(
        referee::topics::kQuickRestartSrv,
        [this](QuickRestart::Request::SharedPtr request,
               QuickRestart::Response::SharedPtr /* response */) {
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
        config_server::topics::kGameSettingsPub, rclcpp::QoS(10).keep_last(1),
        [this](rj_msgs::msg::GameSettings::SharedPtr msg) {
            set_team_color(msg->request_blue_team);
            set_goalie(msg->request_goalie_id);
        });
}

}  // namespace referee