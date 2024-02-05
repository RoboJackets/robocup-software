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

/**
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
    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        DEFAULT,           // Decide what to do
        SEEKING_START,     // Calculate seeking point
        SEEKING,           // Get open
        POSSESSION_START,  // Try to shoot and send pass request
        POSSESSION,        // Holding the ball
        PASSING,           // Getting rid of it
        STEALING,          // Getting the ball
        RECEIVING,         // Getting the ball from a pass
        SHOOTING,           // Winning the game
    };

    /**
     * @return what the state should be right now. called on each get_task tick
     */
    State next_state();

    /**
     * @return the task to execute. called on each get_task tick AFTER next_state()
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    /**
     * @brief This FSM has timeouts for certain states.
     * Ideally, these would not be necessary; as planners get more sophisticated
     * they should not get "stuck". However, empirically, the offense FSM in particular
     * has been observed to deadlock often.
     *
     * One common case is when waiting for a receiver to accept a pass; if no receiver responds,
     * the timeout is necessary. In the future receivers may be able to respond in the negative
     * instead of ignoring the request.
     *
     * The timeouts are a safety mechanism, and should not be the primary reason for a
     * state transition. They are set relatively high for this reason.
     *
     * @return the maximum duration to stay in a given state, or -1 if there is no maximum.
     *
     */
    static constexpr RJ::Seconds timeout(State s) {
        switch (s) {
            case DEFAULT:
                return RJ::Seconds{-1};
            case SEEKING_START:
                return RJ::Seconds{-1};
            case SEEKING:
                return RJ::Seconds{-1};
            case POSSESSION:
                return RJ::Seconds{-1};
            case POSSESSION_START:
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

    // For debugging
    static constexpr std::string_view state_to_name(State s) {

        switch (s) {
            case DEFAULT:
                return "DEFAULT";
            case SEEKING_START:
                return "SEEKING_START";
            case SEEKING:
                return "SEEKING";
            case POSSESSION:
                return "POSSESSION";
            case POSSESSION_START:
                return "POSSESSION_START";
            case PASSING:
                return "PASSING";
            case STEALING:
                return "STEALING";
            case RECEIVING:
                return "RECEIVING";
            case SHOOTING:
                return "SHOOTING";
        }
    }

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

    State current_state_ = State::DEFAULT;

    // The time at which the last state started.
    RJ::Time last_time_;

    /* RoleInterface Members */
    Seeker seeker_;

    /* Constants for State or Task Calculation */

    // These variables are for calculating ball speed when passing
    static constexpr float kFinalBallSpeed{0.0f};

    // Used to tell if we have lost all chance of possessing the ball
    static constexpr double kBallTooFarDist{0.5};

    // Used to tell if the ball is close enough to steal
    static constexpr double kStealBallRadius{0.5};

    // Used to assume we are capable of manipulating the ball
    static constexpr double kOwnBallRadius{kRobotRadius + 0.1};

    /* Utility functions for State or Task Calculation */

    /**
     * @brief Checks if this agent has a good shot, in which case it should shoot instead of
     * passing.
     */
    bool has_open_shot() const;

    /**
     * @brief Calculates the distance of vector from other team's closest robot
     */
    double distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const;

    /**
     * @brief Check if this agent could easily steal the ball
     */
    bool can_steal_ball() const;

    /**
     * @return distance from this agent to ball
     */
    double distance_to_ball() const {
        return last_world_state_->ball.position.dist_to(
            last_world_state_->get_robot(true, robot_id_).pose.position());
    };

    /**
     * @return the target (within the goal) that would be the most clear shot
    */
    rj_geometry::Point calculate_best_shot() const;
};

}  // namespace strategy
