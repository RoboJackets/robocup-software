#pragma once

#include <chrono>
#include <string>
#include <string_view>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rj_common/planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * The Chaser position is a 3-robot support role.
 * It shades between cover and attack support, wins loose balls, and prefers
 * to recycle possession to the dedicated offense robot.
 */
class Chaser : public Position {
public:
    Chaser(int r_id);
    ~Chaser() override = default;
    Chaser(Position&& other);

    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;
    void receive_communication_response(communication::AgentPosResponseWrapper response) override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        POSITIONING,  // Maintain support shape; choose cover or attack support based on play.
        PRESSING,     // Win a loose ball when we are the best non-goalie challenger.
        POSSESSION,   // Secure the ball and look to recycle it to offense.
        PASSING,      // Pass to the offense robot once it has acknowledged.
        RECEIVING,    // Receive or collect an incoming pass.
    };

    State next_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    static constexpr RJ::Seconds timeout(State s) {
        switch (s) {
            case POSITIONING:
                return RJ::Seconds{-1};
            case PRESSING:
                return RJ::Seconds{4};
            case POSSESSION:
                return RJ::Seconds{3};
            case PASSING:
                return RJ::Seconds{5};
            case RECEIVING:
                return RJ::Seconds{5};
        }
    }

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case POSITIONING:
                return "POSITIONING";
            case PRESSING:
                return "PRESSING";
            case POSSESSION:
                return "POSSESSION";
            case PASSING:
                return "PASSING";
            case RECEIVING:
                return "RECEIVING";
        }
    }

    void reset_timeout() { last_time_ = RJ::now(); }

    bool timed_out() const {
        using namespace std::chrono_literals;

        const auto max_time = timeout(current_state_);
        return (max_time > 0s) && (last_time_ + max_time < RJ::now());
    }

    [[nodiscard]] int offense_robot_id() const { return kPrimaryOffenseRobotId; }
    [[nodiscard]] double distance_to_ball() const {
        return last_world_state_->ball.position.dist_to(
            last_world_state_->get_robot(true, robot_id_).pose.position());
    }
    [[nodiscard]] bool ball_in_our_half() const;
    [[nodiscard]] bool should_press() const;
    [[nodiscard]] bool check_if_open(int from_robot_id) const;

    static constexpr int kPrimaryOffenseRobotId = 1;
    static constexpr double kOwnBallRadius = kRobotRadius + 0.1;
    static constexpr double kPressBallRadius = 0.7;
    static constexpr double kBallLostDistance = 0.6;

    State current_state_ = State::POSITIONING;
    RJ::Time last_time_;
    int pass_to_robot_id_ = kPrimaryOffenseRobotId;
    bool pass_request_sent_ = false;
};

}  // namespace strategy
