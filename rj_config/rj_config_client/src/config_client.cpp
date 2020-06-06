#include <rj_config_client/config_client.h>

namespace config_client {
ConfigClient::ConfigClient(const std::string &node_name,
                           const rclcpp::NodeOptions &options)
    : Node{node_name, options} {
  const auto callback = [this](GameSettingsMsg::UniquePtr msg) {
    game_settings_ = *msg;
  };
  subscriber_ = create_subscription<GameSettingsMsg>("/config/game_settings", 1,
                                                     callback);
}
} // namespace config_client
