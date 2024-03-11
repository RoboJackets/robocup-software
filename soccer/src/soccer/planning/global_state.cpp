#include "planning/global_state.hpp"

namespace planning {

GlobalState::GlobalState(rclcpp::Node* node) {
    play_state_sub_ = node->create_subscription<rj_msgs::msg::PlayState>(
        referee::topics::kPlayStateTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::PlayState::SharedPtr state) {  // NOLINT
            {
                auto lock = std::lock_guard(last_play_state_mutex_);
                last_play_state_ = rj_convert::convert_from_ros(*state);
                have_play_state_ = true;
            }
            set_static_obstacles();
        });
    game_settings_sub_ = node->create_subscription<rj_msgs::msg::GameSettings>(
        config_server::topics::kGameSettingsTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::GameSettings::SharedPtr settings) {  // NOLINT
            auto lock = std::lock_guard(last_game_settings_mutex_);
            last_game_settings_ = rj_convert::convert_from_ros(*settings);
        });
    goalie_sub_ = node->create_subscription<rj_msgs::msg::Goalie>(
        referee::topics::kGoalieTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::Goalie::SharedPtr goalie) {  // NOLINT
            auto lock = std::lock_guard(last_goalie_id_mutex_);
            last_goalie_id_ = goalie->goalie_id;
        });
    global_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
        planning::topics::kGlobalObstaclesTopic, rclcpp::QoS(1),
        [this](rj_geometry_msgs::msg::ShapeSet::SharedPtr global_obstacles) {  // NOLINT
            auto lock = std::lock_guard(last_global_obstacles_mutex_);
            last_global_obstacles_ = rj_convert::convert_from_ros(*global_obstacles);
        });
    world_state_sub_ = node->create_subscription<rj_msgs::msg::WorldState>(
        vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
            auto lock = std::lock_guard(last_world_state_mutex_);
            last_world_state_ = rj_convert::convert_from_ros(*world_state);
        });
    field_dimensions_sub_ = node->create_subscription<rj_msgs::msg::FieldDimensions>(
        ::config_server::topics::kFieldDimensionsTopic, rclcpp::QoS(1).transient_local(),
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr msg) {  // NOLINT
            {
                auto lock = std::lock_guard(last_field_dimensions_mutex_);
                last_field_dimensions_ = rj_convert::convert_from_ros(*msg);
                have_field_dimensions_ = true;
            }
            set_static_obstacles();
        });
}

[[nodiscard]] PlayState GlobalState::play_state() const {
    auto lock = std::lock_guard(last_play_state_mutex_);
    return last_play_state_;
}

[[nodiscard]] GameSettings GlobalState::game_settings() const {
    auto lock = std::lock_guard(last_game_settings_mutex_);
    return last_game_settings_;
}
[[nodiscard]] int GlobalState::goalie_id() const {
    auto lock = std::lock_guard(last_goalie_id_mutex_);
    return last_goalie_id_;
}
[[nodiscard]] rj_geometry::ShapeSet GlobalState::global_obstacles() const {
    auto lock = std::lock_guard(last_global_obstacles_mutex_);
    return last_global_obstacles_;
}
[[nodiscard]] rj_geometry::ShapeSet GlobalState::def_area_obstacles() const {
    auto lock = std::lock_guard(last_def_area_obstacles_mutex_);
    return last_def_area_obstacles_;
}
[[nodiscard]] const WorldState* GlobalState::world_state() const {
    auto lock = std::lock_guard(last_world_state_mutex_);
    return &last_world_state_;
}

rj_geometry::ShapeSet GlobalState::create_defense_area_obstacles() {
    // need field dimensions and to be initialized for this to
    // work
    // Create defense areas as rectangular area obstacles
    auto our_defense_area{
        std::make_shared<rj_geometry::Rect>(last_field_dimensions_.our_defense_area())};

    auto our_goal_area{std::make_shared<rj_geometry::Rect>(last_field_dimensions_.our_goal_area())};

    auto their_goal_area{
        std::make_shared<rj_geometry::Rect>(last_field_dimensions_.their_goal_area())};

    //  auto their_goal_area{
    //     std::make_shared<rj_geometry::Rect>(last_field_dimensions_.our_goal_area())};

    // Sometimes there is a greater distance we need to keep:
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area
    bool is_extra_dist_necessary = (last_play_state_.state() == PlayState::State::Stop ||
                                    last_play_state_.restart() == PlayState::Restart::Free);

    // Also add a slack around the box
    float slack_around_box{0.3f};

    auto their_defense_area =
        is_extra_dist_necessary
            ? std::make_shared<rj_geometry::Rect>(
                  last_field_dimensions_.their_defense_area_padded(slack_around_box))
            : std::make_shared<rj_geometry::Rect>(last_field_dimensions_.their_defense_area());

    // Combine both defense areas into ShapeSet
    rj_geometry::ShapeSet def_area_obstacles{};
    def_area_obstacles.add(our_defense_area);
    def_area_obstacles.add(our_goal_area);
    def_area_obstacles.add(their_defense_area);
    def_area_obstacles.add(their_goal_area);

    return def_area_obstacles;
}

void GlobalState::set_static_obstacles() {
    std::scoped_lock lock{last_field_dimensions_mutex_, last_play_state_mutex_,
                          last_def_area_obstacles_mutex_};

    if (have_field_dimensions_ && have_play_state_) {
        // SPDLOG_INFO("Creating new defense obstacles");
        last_def_area_obstacles_ = create_defense_area_obstacles();
    }
}

}  // namespace planning
