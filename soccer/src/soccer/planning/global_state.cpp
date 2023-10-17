#include "global_state.hpp"

namespace planning {

GlobalState::GlobalState(rclcpp::Node* node) {
    play_state_sub_ = node->create_subscription<rj_msgs::msg::PlayState>(
        referee::topics::kPlayStateTopic, rclcpp::QoS(1),
        [&](rj_msgs::msg::PlayState::SharedPtr state) {
            std::lock_guard<std::mutex> lock(last_play_state_mutex_);
            last_play_state_ = rj_convert::convert_from_ros(*state);
        });

    game_settings_sub_ = node->create_subscription<rj_msgs::msg::GameSettings>(
        config_server::topics::kGameSettingsTopic, rclcpp::QoS(1),
        [&](rj_msgs::msg::GameSettings::SharedPtr settings) {
            std::lock_guard<std::mutex> lock(last_game_settings_mutex_);
            last_game_settings_ = rj_convert::convert_from_ros(*settings);
        });

    goalie_sub_ = node->create_subscription<rj_msgs::msg::Goalie>(
        referee::topics::kGoalieTopic, rclcpp::QoS(1), [&](rj_msgs::msg::Goalie::SharedPtr goalie) {
            std::lock_guard<std::mutex> lock(last_goalie_id_mutex_);
            last_goalie_id_ = goalie->goalie_id;
        });

    global_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
        planning::topics::kGlobalObstaclesTopic, rclcpp::QoS(1),
        [&](rj_geometry_msgs::msg::ShapeSet::SharedPtr global_obstacles) {
            std::lock_guard<std::mutex> lock(last_global_obstacles_mutex_);
            last_global_obstacles_ = rj_convert::convert_from_ros(*global_obstacles);
        });

    def_area_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
        planning::topics::kDefAreaObstaclesTopic, rclcpp::QoS(1),
        [&](rj_geometry_msgs::msg::ShapeSet::SharedPtr def_area_obstacles) {
            std::lock_guard<std::mutex> lock(last_def_area_obstacles_mutex_);
            last_def_area_obstacles_ = rj_convert::convert_from_ros(*def_area_obstacles);
        });

    world_state_sub_ = node->create_subscription<rj_msgs::msg::WorldState>(
        vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
        [&](rj_msgs::msg::WorldState::SharedPtr world_state) {
            std::lock_guard<std::mutex> lock(last_world_state_mutex_);
            last_world_state_ = rj_convert::convert_from_ros(*world_state);
        });

    coach_state_sub_ = node->create_subscription<rj_msgs::msg::CoachState>(
        "/strategy/coach_state", rclcpp::QoS(1),
        [&](rj_msgs::msg::CoachState::SharedPtr coach_state) {
            std::lock_guard<std::mutex> lock(last_coach_state_mutex_);
            last_coach_state_ = *coach_state;
        });
}

[[nodiscard]] PlayState GlobalState::play_state() const {
    std::lock_guard<std::mutex> lock(last_play_state_mutex_);
    return last_play_state_;
}

[[nodiscard]] GameSettings GlobalState::game_settings() const {
    std::lock_guard<std::mutex> lock(last_game_settings_mutex_);
    return last_game_settings_;
}

[[nodiscard]] int GlobalState::goalie_id() const {
    std::lock_guard<std::mutex> lock(last_goalie_id_mutex_);
    return last_goalie_id_;
}

[[nodiscard]] rj_geometry::ShapeSet GlobalState::global_obstacles() const {
    std::lock_guard<std::mutex> lock(last_global_obstacles_mutex_);
    return last_global_obstacles_;
}

[[nodiscard]] rj_geometry::ShapeSet GlobalState::def_area_obstacles() const {
    std::lock_guard<std::mutex> lock(last_def_area_obstacles_mutex_);
    return last_def_area_obstacles_;
}

[[nodiscard]] const WorldState GlobalState::world_state() const {
    std::lock_guard<std::mutex> lock(last_world_state_mutex_);
    return last_world_state_;
}

[[nodiscard]] const rj_msgs::msg::CoachState GlobalState::coach_state() const {
    std::lock_guard<std::mutex> lock(last_coach_state_mutex_);
    return last_coach_state_;
}

}  // namespace planning