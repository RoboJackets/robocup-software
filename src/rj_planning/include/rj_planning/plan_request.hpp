#pragma once

#include <map>
#include <memory>
#include <utility>

#include <rj_common/context.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_command.hpp>
#include <rj_common/planning/motion_constraints.hpp>
#include <rj_common/planning/robot_constraints.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/ros_debug_drawer.hpp>
#include <rj_common/world_state.hpp>
#include <rj_planning/obstacle_set.hpp>

#include "rj_planning/global_state.hpp"
#include "rj_planning/trajectory_collection.hpp"

namespace planning {

/**
 * @brief Encapsulates information needed for planner to make a path
 *
 * @details This struct contains ALL information necessary for a single
 * robot path to be planned.
 */
struct PlanRequest {
    PlanRequest(RobotInstant start, MotionCommand command,  // NOLINT
                RobotConstraints constraints, rj_geometry::ShapeSet field_obstacles,
                std::shared_ptr<TrajectoryCollection> planned_trajectories, unsigned shell_id,
                const WorldState* world_state, PlayState play_state,
                const FieldDimensions* field_dimensions, int8_t priority = 0,
                rj_drawing::RosDebugDrawer* debug_drawer = nullptr, bool ball_sense = false,
                float min_dist_from_ball = 0, float kick_speed = 0,
                RobotIntent::TriggerMode trigger_mode = RobotIntent::TriggerMode::STAND_DOWN,
                RobotIntent::DribblerMode dribbler_mode = RobotIntent::DribblerMode::DEFAULT)
        : start(start),
          motion_command(command),  // NOLINT
          constraints(constraints),
          field_obstacles(std::move(field_obstacles)),
          planned_trajectories(planned_trajectories),
          shell_id(shell_id),
          world_state(world_state),
          priority(priority),
          play_state(play_state),
          field_dimensions(field_dimensions),
          debug_drawer(debug_drawer),
          ball_sense(ball_sense),
          min_dist_from_ball(min_dist_from_ball),
          kick_speed(kick_speed),
          trigger_mode(trigger_mode),
          dribbler_mode(dribbler_mode) {}

    /**
     * The robot's starting state.
     */
    RobotInstant start;

    /**
     * The goal to plan for.
     */
    MotionCommand motion_command;

    /**
     * Angular and linear acceleration and velocity constraints on the robot.
     */
    RobotConstraints constraints;

    /**
     * The list of field obstacles including restricted defense areas and gameplay
     * specific obstacles from robot intent
     */
    rj_geometry::ShapeSet field_obstacles;

    /**
     * Trajectories for each of the robots that has already been planned.
     * nullptr for unplanned robots.
     */
    std::shared_ptr<TrajectoryCollection> planned_trajectories;

    /**
     * The robot's shell ID. Used for debug drawing.
     */
    unsigned shell_id;

    /**
     * The state of the world, containing robot and ball states.
     *
     * For obstacle-avoidance purposes, obstacles should be used instead. This
     * can be used for lookup of robots/balls by ID.
     */
    const WorldState* world_state;

    /**
     * The priority of this plan request.
     */
    int8_t priority;

    /**
     * the current PlayState
     */
    PlayState play_state;

    /**
     * the Field Dimensions
     */
    const FieldDimensions* field_dimensions;

    /**
     * Allows debug drawing in the world. If this is nullptr, no debug drawing
     * should be performed.
     */
    rj_drawing::RosDebugDrawer* debug_drawer;

    /**
     * Whether the robot has a ball
     */
    bool ball_sense = false;

    /**
     * How far away to stay from the ball, if the MotionCommand chooses to avoid the ball.
     */
    float min_dist_from_ball = 0;

    /**
     * Kick Speed
     */
    float kick_speed = 0;

    RobotIntent::TriggerMode trigger_mode = RobotIntent::TriggerMode::STAND_DOWN;
    RobotIntent::DribblerMode dribbler_mode = RobotIntent::DribblerMode::DEFAULT;
};

/**
 * Fill the obstacle fields.
 *
 * @param in the plan request.
 * @param out_obstacles an (empty) ObstacleSet to be populated.
 *  This will be filled with field obstacles, the ball,
 *  opponent robots, and our robots.
 * @param avoid_ball whether to avoid the ball. If this is true, the ball
 *  will be included as an obstacle to avoid.
 */
void fill_obstacles(const PlanRequest& in, ObstacleSet& out_obstacles, bool avoid_ball);

// Obstacle padding constants
namespace obstacle {
constexpr float kVelocityScaling = 0.3f;          // How far ahead to project velocity for padding
constexpr float kVelocityWidthScaling = 0.1f;     // How much to inflate width based on velocity
constexpr float kPaddingRadiusMultiplier = 1.25f;  // Multiplier for obstacle padding radius
}  // namespace obstacle

/**
 * Create a static robot obstacle (circular).
 *
 * @param pos The position of the robot
 * @return Shared pointer to the created Obstacle
 */
inline std::shared_ptr<Obstacle> make_robot_obstacle(rj_geometry::Point pos) {
    auto circle = std::make_shared<rj_geometry::Circle>(pos, kRobotRadius);
    return std::make_shared<Obstacle>(circle, circle);
}

/**
 * Create a moving robot obstacle with stadium-shaped padding.
 * The padding shape is inflated based on velocity to account for movement.
 *
 * @param pos The position of the robot
 * @param vel The velocity of the robot
 * @return Shared pointer to the created Obstacle
 */
inline std::shared_ptr<Obstacle> make_moving_robot_obstacle(rj_geometry::Point pos,
                                                            rj_geometry::Point vel) {
    auto obs_circle = std::make_shared<rj_geometry::Circle>(pos, kRobotRadius);

    // Create stadium-shaped padding based on velocity
    auto padding = std::make_shared<rj_geometry::StadiumShape>(
        pos, pos + vel * obstacle::kVelocityScaling,
        obstacle::kPaddingRadiusMultiplier * kRobotRadius +
            (vel.mag() * obstacle::kVelocityWidthScaling));

    return std::make_shared<Obstacle>(obs_circle, padding);
}

/**
 * Create a ball obstacle (circular).
 *
 * @param pos The position of the ball
 * @param extra_radius Additional radius to add beyond the default ball avoidance distance
 * @return Shared pointer to the created Obstacle
 */
inline std::shared_ptr<Obstacle> make_ball_obstacle(rj_geometry::Point pos,
                                                    float extra_radius = 0.0f) {
    float radius = kBallRadius + kAvoidBallDistance + extra_radius;
    auto circle = std::make_shared<rj_geometry::Circle>(pos, radius);
    return std::make_shared<Obstacle>(circle, circle);
}

}  // namespace planning
