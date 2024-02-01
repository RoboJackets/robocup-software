#pragma once

#include <chrono>
#include <cmath>

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
#include "seeker.hpp"

namespace strategy {

/*
 * The Offense position handles general offensive tasks, like passing to
 * teammates, moving to get open, or shooting on goal.
 */
class Offense : public Position {
public:
    Offense(int r_id);
    ~Offense() override = default;
    
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        DEFAULT, // Decide what to do
        SEEKING, // Get open
        POSSESSION, // Holding the ball
        PASSING, // Getting rid of it
        STEALING, // Getting the ball
        RECEIVING, // Getting the ball from a pass
        SHOOTING // Winning the game
    };

    // The longest amount of time we should be 
    static constexpr RJ::Seconds timeout(State s) {
        switch (s) {
            case DEFAULT:
                return RJ::Seconds{-1};
            case SEEKING:
                return RJ::Seconds{-1};
            case POSSESSION:
                return RJ::Seconds{-1};
            case PASSING:
                return RJ::Seconds{5};
            case STEALING:
                return RJ::Seconds{5};
            case RECEIVING:
                return RJ::Seconds{5};
            case SHOOTING:
                return RJ::Seconds{5};
        }
    }

    void reset_timeout();

    bool timed_out();

    RJ::Time last_time_;

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of the offensive agent (state machine)
    State current_state_ = State::DEFAULT;

    // These variables are for calculating ball speed when passing
    static constexpr float kFinalBallSpeed{0.0f};


    // The following functions are calculations to aid state transition or MotionCommand creation.

    /**
    * @brief Checks if this agent has a good shot, in which case it should shoot instead of passing.
    */
    bool has_open_shot();

    /**
     * @brief Calculates the distance of vector from other team's closest robot
     */
    double distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point heaad);
};

}  // namespace strategy
