#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <context.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/srv/plan_hypothetical_path.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "node.hpp"
#include "world_state.hpp"
#include "game_settings.hpp"
#include "rj_geometry/shape_set.hpp"
#include "game_state.hpp"

namespace planning {

class GlobalState {
public:
    GlobalState(rclcpp::Node* node);

    [[nodiscard]] PlayState play_state() const;
    [[nodiscard]] GameSettings game_settings() const;
    [[nodiscard]] int goalie_id() const;
    [[nodiscard]] rj_geometry::ShapeSet global_obstacles() const;
    [[nodiscard]] rj_geometry::ShapeSet def_area_obstacles() const;
    [[nodiscard]] WorldState world_state() const;
    [[nodiscard]] rj_msgs::msg::CoachState coach_state() const;

private:
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr def_area_obstacles_sub_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::CoachState>::SharedPtr coach_state_sub_;

    PlayState last_play_state_;
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
} // namespace planning