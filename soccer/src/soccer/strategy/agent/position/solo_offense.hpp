#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

class SoloOffense : public Position {
public:
    SoloOffense(const Position& other);
    SoloOffense(int r_id);
    ~SoloOffense() override = default;
    SoloOffense(const SoloOffense& other) = default;
    SoloOffense(SoloOffense&& other) = default;

    std::string get_current_state() override;

private:
    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        DEFAULT,
        TO_BALL,
        GATHER_STEP,
        PIVOT,
        SIDE_STEP,
        AIM_AND_SHOOT,
        MARKER,
    };
    State current_state_ = DEFAULT;

    /**
     * @brief This FSM has timeouts for certain states.
     *
     * These timeouts are a safety mechanism against unpredictable enemy behavior.
     * In blind, fixed-target motion commands, we may be denied by opponents. 
     * These timeouts explicitly act as give up points (at which point the attacker progresses to the next phase).
     *
     * @return the maximum duration to stay in a given state, or -1 if there is no maximum.
     */
    static constexpr RJ::Seconds timeout(State s) {
        switch (s) {
            case DEFAULT: return RJ::Seconds{-1};
            case TO_BALL: return RJ::Seconds{-1};
            case GATHER_STEP: return RJ::Seconds{3};
            case PIVOT: return RJ::Seconds{-1};
            case SIDE_STEP: return RJ::Seconds{5};
            case AIM_AND_SHOOT: return RJ::Seconds{-1};
            case MARKER: return RJ::Seconds{-1};
        }
    }
    // The time at which the last state started.
    RJ::Time last_time_;
    /**
     * @brief Reset the timeout for the current state
     */
    void reset_timeout() {
        // Defined here so it can be inlined
        last_time_ = RJ::now();
    }
    /**
     * @return if the current state has timed out
     */
    bool timed_out() const {
        // Defined here so it can be inlined
        using namespace std::chrono_literals;

        const auto max_time = timeout(current_state_);

        return (max_time > 0s) && (last_time_ + max_time < RJ::now());
    };


    
    rj_geometry::Point cached_ball_pos_;
    rj_geometry::Point get_ball_pos() const;

    // rj_geometry::Point target_;

    int marking_id_;

    static constexpr double kGatherLength = kRobotRadius;
    rj_geometry::Point gather_target_;
    rj_geometry::Point calculate_gather() const;
    rj_geometry::Point juke_target_;
    rj_geometry::Point calculate_juke() const;
    rj_geometry::Point shot_target_;
    rj_geometry::Point calculate_best_shot() const;

    /**
     * @return whether the ball is likely to be in the dribbler
     */
    bool ball_in_dribbler() const;
    static constexpr double kDribblerTolerance = 0.1;  // Tight region such that ball is probably in the dribbler (and thus the robot has possession)
    static constexpr double kWideRobotTolerance = 3*kRobotRadius; // Wide region about robot center where posession is loosely assumed

    /**
     * @return whether to go for an attack (don't if another teammate is touching ball)
     */
    bool teammate_attacking() const;

    /**
     * @return the id of the opponent to mark, if any
     */
    int find_mark() const;
    
    /**
     * @return whether the point is in an area that non-goalies cannot reach.
     */
    bool point_in_red(rj_geometry::Point concerned_point) const;

    /**
     * @return what the state should be right now. called on each get_task tick
     */
    State next_state();

    /**
     * @return the task to execute. called on each get_task tick AFTER next_state()
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    double distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const;
};

}  // namespace strategy
