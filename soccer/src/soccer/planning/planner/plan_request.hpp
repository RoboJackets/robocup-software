#pragma once

#include <map>
#include <memory>
#include <utility>

#include <planning/motion_constraints.hpp>
#include <planning/planner/motion_command.hpp>
#include <planning/trajectory.hpp>

#include "../global_state.hpp"
#include "context.hpp"
#include "planning/dynamic_obstacle.hpp"
#include "planning/instant.hpp"
#include "planning/robot_constraints.hpp"
#include "planning/trajectory_collection.hpp"
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
                rj_geometry::ShapeSet virtual_obstacles, TrajectoryCollection* planned_trajectories,
                unsigned shell_id, const WorldState* world_state, PlayState play_state,
                int8_t priority = 0, rj_drawing::RosDebugDrawer* debug_drawer = nullptr,
                bool ball_sense = false, float min_dist_from_ball = 0, float dribbler_speed = 0)
        : start(start),
          motion_command(command),  // NOLINT
          constraints(constraints),
          field_obstacles(std::move(field_obstacles)),
          virtual_obstacles(std::move(virtual_obstacles)),
          planned_trajectories(planned_trajectories),
          shell_id(shell_id),
          priority(priority),
          world_state(world_state),
          play_state(play_state),
          debug_drawer(debug_drawer),
          ball_sense(ball_sense),
          min_dist_from_ball(min_dist_from_ball),
          dribbler_speed(dribbler_speed) {}

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
    TrajectoryCollection* planned_trajectories;

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
     * Allows debug drawing in the world. If this is nullptr, no debug drawing
     * should be performed.
     */
    rj_drawing::RosDebugDrawer* debug_drawer;

    // Whether the robot has a ball
    bool ball_sense = false;
    /**
     * How far away to stay from the ball, if the MotionCommand chooses to avoid the ball.
     */
    float min_dist_from_ball = 0;

    /**
     * Dribbler Speed
     */
    float dribbler_speed = 0;
};

/**
 * Create static circle obstacle for one of the robots.
 *
 * @param robot current RobotState to make an obstacle for
 * @param obs_center outparam, Point representing new obstacle's center
 * @param obs_radius outparam, double for new obstacle's radius
 *
 * Shift the circle off-center depending on their robot's current velocity.
 * Also, inflate circle radius based on robot velocity.
 *
 * (see section 2.5:
 * https://ssl.robocup.org/wp-content/uploads/2019/03/2019_ETDP_TIGERs_Mannheim.pdf)
 *
 * obs_center_shift = 0.1 (robot rads per m/s speed):
 * At 0 m/s, obstacle center = opp robot position
 * 1 m/s = center shifted by 0.1 * robot radius in dir of travel
 * 2 m/s = center shifted by 2 * 0.1 * robot rad in dir of travel
 * ...
 *
 * obs_radius_inflation = 0.1 (robot rads per m/s speed):
 * At 0 m/s, obs_radius = robot radius
 * 1 m/s = obs_radius = robot radius + 0.1 * robot radius
 * 2 m/s = obs_radius = robot radius + 2 * 0.1 * robot radius
 * ...
 *
 * Numbers tuned by looking at output of planning/test_scripts/visualize_obs.py.
 *
 */
void fill_robot_obstacle(const RobotState& robot, rj_geometry::Point& obs_center,
                         double& obs_radius);

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
 * @param avoid_ball whether to avoid the ball. If this is true, out_ball_trajectory
 *  should point to a valid trajectory.
 * @param ball_trajectory temporary storage for the ball trajectory. This must
 *  outlive the usage of out_dynamic. If avoid_ball == false, this should be
 *  nullptr.
 */
void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory = nullptr);

}  // namespace planning
