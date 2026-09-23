#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_command.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/role_interface.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {

/**
 * This position stays 0.15 meters behind the ball at all times.
 */
class PenaltyPlayer : public Position {
public:
    PenaltyPlayer(int r_id);
    ~PenaltyPlayer() = default;
    PenaltyPlayer(Position&& other);

    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_pass() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_pass_ball() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string(state_to_name(latest_state_));
    }

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State { START, SMALL_KICK, LINE_UP, SHOOTING_START, SHOOTING };

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case START:
                return "START";
            case SMALL_KICK:
                return "SMALL_KICK";
            case LINE_UP:
                return "LINE_UP";
            case SHOOTING_START:
                return "SHOOTING_START";
            case SHOOTING:
                return "SHOOTING";
        }
        return "UNKNOWN";
    }

    static constexpr double kDistanceToGoalThreshold{6};

    State update_state();

    /**
     * @brief Calculates the distance of vector from other team's closest robot
     */
    double distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const;

    double distance_to_ball() const {
        return last_world_state_->ball.position.dist_to(
            last_world_state_->get_robot(true, robot_id_).pose.position());
    };

    double distance_from_enemy_goal() const {
        return last_world_state_->get_robot(true, robot_id_)
            .pose.position()
            .dist_to(field_dimensions_.their_goal_loc());
    };

    /**
     * @return the target (within the goal) that would be the most clear shot
     */
    rj_geometry::Point calculate_best_shot() const;

    // where to kick to
    rj_geometry::Point target_;

    /*
     * Based on the Goalie's current state, send a motion_command
     * to the planner node.
     *
     * @param intent RobotIntent to add the desired motion_command
     * to
     *
     * @return matching return type of derived_get_task() (see
     * above)
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of Goalie (state machine)
    State latest_state_ = START;
};

}  // namespace strategy
