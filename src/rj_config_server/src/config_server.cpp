#include <rclcpp/rclcpp.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_config_server/config_server.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_utils/logging_macros.hpp>

namespace config_server {
ConfigServer::ConfigServer(const rclcpp::NodeOptions& node_options,
                           const GameSettingsMsg& game_settings)
    : Node{"config_server", node_options},
      game_settings_{game_settings},
      field_dimensions_{rj_convert::convert_to_ros(FieldDimensions::kDefaultDimensions)} {
    const auto latching_qos = rclcpp::QoS(1).transient_local();
    // Game Settings
    game_settings_publisher_ =
        create_publisher<GameSettingsMsg>(topics::kGameSettingsTopic, latching_qos);

    // NOLINTS below are to disable performance-unnecessary-value-param
    const auto game_settings_cb = [this](const SetGameSettingsSrvReqPtr request,    // NOLINT
                                         SetGameSettingsSrvRespPtr /*response*/) {  // NOLINT
        set_game_settings_callback(request->game_settings);
    };
    game_settings_server_ =
        create_service<SetGameSettingsSrv>(topics::kGameSettingsSrv, game_settings_cb);

    // Field Dimensions
    field_dimensions_publisher_ =
        create_publisher<FieldDimensionsMsg>(topics::kFieldDimensionsTopic, latching_qos);

    // NOLINTS below are to disable performance-unnecessary-value-param
    const auto field_dimensions_cb = [this](const SetFieldDimensionsSrvReqPtr request,    // NOLINT
                                            SetFieldDimensionsSrvRespPtr /*response*/) {  // NOLINT
        set_field_dimensions_callback(request->field_dimensions);
    };
    field_dimensions_server_ =
        create_service<SetFieldDimensionsSrv>(topics::kFieldDimensionsSrv, field_dimensions_cb);

    broadcast_game_settings();
    broadcast_field_dimensions();

    EZ_INFO("config_server is up!");
}

void ConfigServer::broadcast_game_settings() { game_settings_publisher_->publish(game_settings_); }

void ConfigServer::set_game_settings_callback(const GameSettingsMsg& msg) {
    game_settings_ = msg;
    broadcast_game_settings();
}

void ConfigServer::broadcast_field_dimensions() {
    field_dimensions_publisher_->publish(field_dimensions_);
}

void ConfigServer::set_field_dimensions_callback(const FieldDimensionsMsg& msg) {
    if (field_dimensions_ != msg) {
        field_dimensions_ = msg;
        broadcast_field_dimensions();
    }
}

}  // namespace config_server

using config_server::ConfigServer;
using config_server::GameSettingsMsg;

/**
 * Parse game settings from the arguments. TODO(#1592): Unify this with the parsing in main().
 * @param args the arguments, with ROS args removed.
 * @return the game settings struct.
 */
GameSettingsMsg parse_game_settings(const std::vector<std::string>& args) {
    GameSettingsMsg game_settings;

    for (size_t i = 1; i < args.size(); i++) {
        const std::string& arg = args.at(i);
        if (arg == "-b") {
            game_settings.request_blue_team = true;
        } else if (arg == "-y") {
            game_settings.request_blue_team = false;
        } else if (arg == "-sim") {
            game_settings.simulation = false;
        } else if (arg == "-defend") {
            i++;
            const std::string& direction = args.at(i);
            if (direction == "plus") {
                game_settings.defend_plus_x = true;
            } else if (direction == "minus") {
                game_settings.defend_plus_x = false;
            } else {
                throw std::invalid_argument(fmt::format("Invalid defend direction: {}", direction));
            }
        }
    }
    return game_settings;
}

int main(int argc, char* argv[]) {
    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    try {
        GameSettingsMsg game_settings = parse_game_settings(args);
        rclcpp::spin(std::make_shared<ConfigServer>(rclcpp::NodeOptions(), game_settings));
    } catch (const std::invalid_argument& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    rclcpp::shutdown();

    return 0;
}
