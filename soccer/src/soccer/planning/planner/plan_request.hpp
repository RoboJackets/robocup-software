#pragma once

#include <map>
#include <memory>
#include <utility>

#include <planning/motion_constraints.hpp>
#include <planning/planner/motion_command.hpp>
#include <planning/trajectory.hpp>

#include "context.hpp"
#include "planning/dynamic_obstacle.hpp"
#include "planning/instant.hpp"
#include "planning/robot_constraints.hpp"
#include "ros_debug_drawer.hpp"
#include "world_state.hpp"

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
                rj_geometry::ShapeSet virtual_obstacles,
                std::array<const Trajectory*, kNumShells> planned_trajectories, unsigned shell_id,
                const WorldState* world_state, int8_t priority = 0,
                rj_drawing::RosDebugDrawer* debug_drawer = nullptr, bool ball_sense = false)
        : start(start),
          motion_command(command),  // NOLINT
          constraints(constraints),
          field_obstacles(std::move(field_obstacles)),
          virtual_obstacles(std::move(virtual_obstacles)),
          planned_trajectories(planned_trajectories),
          shell_id(shell_id),
          priority(priority),
          world_state(world_state),
          debug_drawer(debug_drawer),
          ball_sense(ball_sense) {}

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
     * The list of field obstacles.
     */
    rj_geometry::ShapeSet field_obstacles;

    /**
     * The list of "virtual" obstacles, generated by gameplay representing soft
     * constraints.
     */
    rj_geometry::ShapeSet virtual_obstacles;

    /**
     * Trajectories for each of the robots that has already been planned.
     * nullptr for unplanned robots.
     */
    std::array<const Trajectory*, kNumShells> planned_trajectories;

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
     * Allows debug drawing in the world. If this is nullptr, no debug drawing
     * should be performed.
     */
    rj_drawing::RosDebugDrawer* debug_drawer;

    // Whether the robot has a ball
    bool ball_sense = false;
};

/**
 * Create static circle obstacle for one of the robots.
 *
 * Shift the circle off-center depending on their robot's current velocity.
 * Also, inflate circle radius based on robot velocity.
 *
 * (see section 2.5:
 * https://ssl.robocup.org/wp-content/uploads/2019/03/2019_ETDP_TIGERs_Mannheim.pdf)
 *
 * At 0 m/s, obstacle center = opp robot position
 * 1 m/s = shifted by 0.2 robot rad in dir of travel
 * 2 m/s = shifted by 0.4 robot rad in dir of travel
 * (linearly increases)
 *
 * At 0 m/s, safety margin = 90% robot radius
 * 1 m/s = 110%
 * 2 m/s = 130%
 * (linearly increases)
 * @param robot ptr to robot that needs obstacle made
 * @param obs_center center Point of new obs
 * @param obs_radius radius of new obs
 */
void calc_static_robot_obs(const RobotState& robot, std::shared_ptr<rj_geometry::Circle>& obs_ptr);

/**
 * Fill the obstacle fields.
 *
 * @param in the plan request.
 * @param out_static an (empty) vector of static obstacles to be populated.
 *  This will be filled with field obstacles, local (virtual) obstacles,
 *  opponent robots, and our robots that have not yet been planned.
 * @param out_dynamic an (empty) vector of dynamic obstacles to be populated.
 *  This will be filled with trajectories for our robots that have already been
 *  planned.
 * @param avoid_ball whether or not to avoid the ball. If this is true,
 *  out_ball_trajectory should point to a valid trajectory.
 * @param ball_trajectory temporary storage for the ball trajectory. This must
 *  outlive the usage of out_dynamic. If avoid_ball == false, this should be
 *  nullptr.
 */
void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory = nullptr);

}  // namespace planning
