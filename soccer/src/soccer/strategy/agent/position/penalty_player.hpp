#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {

/**
 * This position stays 0.15 meters behind the ball at all times.
 */
class PenaltyPlayer : public Position {
public:
    PenaltyPlayer(int r_id);
    ~PenaltyPlayer() = default;
    PenaltyPlayer(const Position& other);

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

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State { LINE_UP, SHOOTING_START, SHOOTING };

    static constexpr double kOwnBallRadius{kRobotRadius + 0.1};

    State update_state();

    /**
     * @brief Calculates the distance of vector from other team's closest robot
     */
    double distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const;

    double distance_to_ball() const {
        return last_world_state_->ball.position.dist_to(
            last_world_state_->get_robot(true, robot_id_).pose.position());
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
    State latest_state_ = LINE_UP;
};

}  // namespace strategy
