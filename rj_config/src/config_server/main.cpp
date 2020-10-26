#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

#include <config_server/config_server.hpp>

using config_server::ConfigServer;
using config_server::GameSettingsMsg;

/**
 * Parse game settings from the arguments. TODO: Unify this with the parsing in main().
 * @param args the arguments, with ROS args removed.
 * @return the game settings struct.
 */
GameSettingsMsg parse_game_settings(const std::vector<std::string>& args) {
    GameSettingsMsg game_settings;
    for (int i = 1; i < args.size(); i++) {
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

int main(int argc, char** argv) {
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
