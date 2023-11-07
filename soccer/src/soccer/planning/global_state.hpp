#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>
#include <rj_geometry/shape_set.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/game_settings.hpp>
#include <rj_msgs/msg/goalie.hpp>

#include "game_settings.hpp"
#include "game_state.hpp"
#include "world_state.hpp"

namespace planning {

/**
 * Aggregates info from other ROS nodes into an unchanging "global state" for
 * each planner. Read-only (from PlannerNode's perspective).
 *
 * ("Global state" in quotes since many of these fields can be changed by other
 * nodes; however, to PlannerNode these are immutable.)
 */
class GlobalState {
public:
    GlobalState(rclcpp::Node* node) {
        play_state_sub_ = node->create_subscription<rj_msgs::msg::PlayState>(
            referee::topics::kPlayStateTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::PlayState::SharedPtr state) {  // NOLINT
                last_play_state_ = rj_convert::convert_from_ros(*state);
            });
        game_settings_sub_ = node->create_subscription<rj_msgs::msg::GameSettings>(
            config_server::topics::kGameSettingsTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::GameSettings::SharedPtr settings) {  // NOLINT
                last_game_settings_ = rj_convert::convert_from_ros(*settings);
            });
        goalie_sub_ = node->create_subscription<rj_msgs::msg::Goalie>(
            referee::topics::kGoalieTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::Goalie::SharedPtr goalie) {  // NOLINT
                last_goalie_id_ = goalie->goalie_id;
            });
        global_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
            planning::topics::kGlobalObstaclesTopic, rclcpp::QoS(1),
            [this](rj_geometry_msgs::msg::ShapeSet::SharedPtr global_obstacles) {  // NOLINT
                last_global_obstacles_ = rj_convert::convert_from_ros(*global_obstacles);
            });
        def_area_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
            planning::topics::kDefAreaObstaclesTopic, rclcpp::QoS(1),
            [this](rj_geometry_msgs::msg::ShapeSet::SharedPtr def_area_obstacles) {  // NOLINT
                last_def_area_obstacles_ = rj_convert::convert_from_ros(*def_area_obstacles);
            });
        world_state_sub_ = node->create_subscription<rj_msgs::msg::WorldState>(
            vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
                last_world_state_ = rj_convert::convert_from_ros(*world_state);
            });
        coach_state_sub_ = node->create_subscription<rj_msgs::msg::CoachState>(
            "/strategy/coach_state", rclcpp::QoS(1),
            [this](rj_msgs::msg::CoachState::SharedPtr coach_state) {  // NOLINT
                last_coach_state_ = *coach_state;
            });
    }

    [[nodiscard]] PlayState play_state() const { return last_play_state_; }
    [[nodiscard]] GameSettings game_settings() const { return last_game_settings_; }
    [[nodiscard]] int goalie_id() const { return last_goalie_id_; }
    [[nodiscard]] rj_geometry::ShapeSet global_obstacles() const { return last_global_obstacles_; }
    [[nodiscard]] rj_geometry::ShapeSet def_area_obstacles() const {
        return last_def_area_obstacles_;
    }
    [[nodiscard]] const WorldState* world_state() const { return &last_world_state_; }
    [[nodiscard]] const rj_msgs::msg::CoachState coach_state() const { return last_coach_state_; }

private:
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr def_area_obstacles_sub_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::CoachState>::SharedPtr coach_state_sub_;

    PlayState last_play_state_ = PlayState::halt();
    GameSettings last_game_settings_;
    int last_goalie_id_;
    rj_geometry::ShapeSet last_global_obstacles_;
    rj_geometry::ShapeSet last_def_area_obstacles_;
    WorldState last_world_state_;
    rj_msgs::msg::CoachState last_coach_state_;
};

}  // namespace planning
