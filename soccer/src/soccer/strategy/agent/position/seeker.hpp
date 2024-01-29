#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {

/*
 * The Seeker role provides the implementation for a offensive robot that
 * is trying to get open, so that they can receive a pass
 */
class Seeker : public RoleInterface {
public:
    Seeker(int robot_id);
    ~Seeker() = default;
    Seeker(const Seeker& other) = default;
    Seeker(Seeker&& other) = default;
    Seeker& operator=(const Seeker& other) = default;
    Seeker& operator=(Seeker&& other) = default;

    /**
     * @brief  Returns a seeker behavior which aims to get open
     *
     * @param intent The RobotIntent to add the movement to
     * @param world_state The current WorldState
     * @param field_dimensions The dimensions of the field
     *
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* world_state,
                                        FieldDimensions field_dimensions) override;

private:
    // The seeker's id
    int robot_id_;
    // The taret point to move to
    rj_geometry::Point target_pt_{0.0, 0.0};

    /**
     * @brief Returns the point which is most 'open'
     *
     * @param world_state The current WorldState
     * @param current_position The current position of the seeker
     * @param field_dimensions The dimensions of the field
     *
     * @return rj_geometry::Point The target point
     */
    rj_geometry::Point get_open_point(const WorldState* world_state,
                                      rj_geometry::Point current_position,
                                      const FieldDimensions& field_dimensions) const;

    /**
     * @brief Calculates which point is the best by iteratively searching around the robot
     *
     * @param current_prec A double that represents how far away to look from the robot
     * @param min_prec A double that represents the minimum distance to look from the robot
     * @param current_point The robot's current position
     * @param world_state The current WorldState
     * @param field_dimensions The dimensions of the field
     *
     * @return rj_geometry::Point The best point found
     */
    rj_geometry::Point calculate_open_point(double current_prec, double min_prec,
                                            rj_geometry::Point current_point,
                                            const WorldState* world_state,
                                            const FieldDimensions& field_dimensions) const;

    /**
     * @brief Corrects the point to be within the field
     *
     * @param point The point to correct
     * @param field_dimensions The dimensions of the field
     *
     * @return rj_geometry::Point The corrected point
     */
    rj_geometry::Point correct_point(rj_geometry::Point point, const FieldDimensions& field_dimensions) const;

    /**
     * @brief Calculates how 'good' a target point is
     *
     * @param ball_pos The current position of the ball
     * @param current_point The point that is being evaluated
     * @param world_state The current world state
     *
     * @return double The evaluation of that target point
     */
    static double eval_point(rj_geometry::Point ball_pos, rj_geometry::Point current_point,
                      const WorldState* world_state);
};

}  // namespace strategy
