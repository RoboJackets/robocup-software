#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(const Position& other);

    void derived_acknowledge_pass() override;
    
    void derived_pass_ball() override;
    
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

private:

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        RUNNING_TO_CORNER_1,
        RUNNING_TO_CORNER_2,
        RUNNING_TO_CORNER_3,
        RUNNING_TO_CORNER_4,
    };

    State next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    void initialize_running_path();

    bool has_reached_target() const;

    rj_geometry::Point get_current_target() const;

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case RUNNING_TO_CORNER_1:
                return "RUNNING_TO_CORNER_1";
            case RUNNING_TO_CORNER_2:
                return "RUNNING_TO_CORNER_2";
            case RUNNING_TO_CORNER_3:
                return "RUNNING_TO_CORNER_3";
            case RUNNING_TO_CORNER_4:
                return "RUNNING_TO_CORNER_4";
        }
    }

    State current_state_ = State::RUNNING_TO_CORNER_1;

    std::vector<rj_geometry::Point> running_path_;

    static constexpr double kReachedThreshold = 0.2;

    static constexpr double kRunningRectWidthRatio = 0.6;
    static constexpr double kRunningRectLengthRatio = 0.8;
};

}