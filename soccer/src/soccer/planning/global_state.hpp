#pragma once

#include <mutex>

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
    GlobalState(rclcpp::Node* node);

    [[nodiscard]] PlayState play_state() const;
    [[nodiscard]] GameSettings game_settings() const;
    [[nodiscard]] int goalie_id() const;
    [[nodiscard]] rj_geometry::ShapeSet global_obstacles() const;
    [[nodiscard]] rj_geometry::ShapeSet def_area_obstacles() const;
    [[nodiscard]] const WorldState* world_state() const;
    [[nodiscard]] const rj_msgs::msg::CoachState coach_state() const;

private:
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr def_area_obstacles_sub_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::CoachState>::SharedPtr coach_state_sub_;

    PlayState last_play_state_ = PlayState::halt();
    mutable std::mutex last_play_state_mutex_;
    GameSettings last_game_settings_;
    mutable std::mutex last_game_settings_mutex_;
    int last_goalie_id_;
    mutable std::mutex last_goalie_id_mutex_;
    rj_geometry::ShapeSet last_global_obstacles_;
    mutable std::mutex last_global_obstacles_mutex_;
    rj_geometry::ShapeSet last_def_area_obstacles_;
    mutable std::mutex last_def_area_obstacles_mutex_;
    WorldState last_world_state_;
    mutable std::mutex last_world_state_mutex_;
    rj_msgs::msg::CoachState last_coach_state_;
    mutable std::mutex last_coach_state_mutex_;
};

}  // namespace planning
