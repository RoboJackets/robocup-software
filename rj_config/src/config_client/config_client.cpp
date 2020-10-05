#include <config_client/config_client.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_utils/logging_macros.hpp>

namespace config_client {
ConfigClient::ConfigClient(rclcpp::Node* node)
    : node_{node}, game_settings_{std::nullopt}, field_dimensions_{std::nullopt} {
    const auto latching_qos = rclcpp::QoS(1).transient_local();

    const auto game_settings_cb = [this](GameSettingsMsg::UniquePtr msg) {
        std::lock_guard<std::mutex> guard{mutex_};
        game_settings_ = *msg;
    };
    game_settings_sub_ = node->create_subscription<GameSettingsMsg>(
        config_server::topics::kGameSettingsPub, latching_qos, game_settings_cb);
    game_settings_client_ =
        node->create_client<SetGameSettingsSrv>(config_server::topics::kGameSettingsSrv);

    const auto field_dimensions_cb = [this](FieldDimensionsMsg::UniquePtr msg) {
        std::lock_guard<std::mutex> guard{mutex_};
        field_dimensions_ = *msg;
    };
    field_dimensions_sub_ = node->create_subscription<FieldDimensionsMsg>(
        config_server::topics::kFieldDimensionsPub, latching_qos, field_dimensions_cb);
    field_dimensions_client_ =
        node->create_client<SetFieldDimensionsSrv>(config_server::topics::kFieldDimensionsSrv);
}

bool ConfigClient::connected() const {
    const bool has_game_settings = game_settings_.has_value();
    const bool has_field_dimensions = field_dimensions_.has_value();

    const bool have_msg = has_game_settings && has_field_dimensions;

    static bool prev_connected = false;

    if (!have_msg) {
        auto& clk = *node_->get_clock();
        const auto throttle_ms = 1000;
        RJ_INFO_STREAM_THROTTLE(node_->get_logger(), clk, throttle_ms,
                                "[ConfigClient] Waiting on ConfigServer...");
    }

    if (!prev_connected && have_msg) {
        prev_connected = true;
        RJ_INFO(node_->get_logger(), "[ConfigClient] Connected!");
    }

    return have_msg;
}

bool ConfigClient::connected_threaded() const {
    std::lock_guard<std::mutex> guard{mutex_};
    return connected();
}

void ConfigClient::update_game_settings(const GameSettingsMsg& msg) {
    SetGameSettingsReq::SharedPtr request = std::make_shared<SetGameSettingsReq>();
    request->game_settings = msg;

    game_settings_client_->async_send_request(request);
}

void ConfigClient::update_field_dimensions(const FieldDimensionsMsg& msg) {
    SetFieldDimensionsReq::SharedPtr request = std::make_shared<SetFieldDimensionsReq>();
    request->field_dimensions = msg;
    field_dimensions_client_->async_send_request(request);
}

bool ConfigClient::wait_until_connected() const {
    constexpr std::chrono::milliseconds kSleepDuration{100};
    while (rclcpp::ok()) {
        if (connected_threaded()) {
            return true;
        }
        rclcpp::sleep_for(kSleepDuration);
    }
    return false;
}
}  // namespace config_client
