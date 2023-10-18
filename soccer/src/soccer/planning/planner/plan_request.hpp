#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <planning/motion_constraints.hpp>
#include <planning/planner/motion_command.hpp>
#include <planning/trajectory.hpp>

#include "context.hpp"
#include "planning/dynamic_obstacle.hpp"
#include "planning/global_state.hpp"
#include "planning/instant.hpp"
#include "planning/robot_constraints.hpp"
#include "planning/trajectory_collection.hpp"
#include "robot_intent.hpp"
#include "ros_debug_drawer.hpp"
#include "world_state.hpp"
#include "planning/global_state.hpp"

namespace planning {

/**
 * @brief Encapsulates information needed for planner to make a path
 *
 * @details This struct contains ALL information necessary for a single
 * robot path to be planned.
 */
struct PlanRequest {
    PlanRequest(const RobotIntent& intent, const GlobalState& global_state,
                rj_drawing::RosDebugDrawer* debug_draw);

    /**
     * The robot's starting state.
     */
    RobotInstant start{};

    /**
     * The goal to plan for.
     */
    MotionCommand motion_command{};

    /**
     * Angular and linear acceleration and velocity constraints on the robot.
     */
    RobotConstraints constraints{};

    /**
     * A vector of dynamic obstacles. Filled with trajectories for our robots that have already been
     * planned
     */
    std::vector<DynamicObstacle> dynamic_obstacles{};

    /**
     * Static obstacles. This will be filled with field obstacles, local (virtual) obstacles,
     * opponent robots, and our robots that have not yet been planned.
     */
    rj_geometry::ShapeSet static_obstacles{};

    /**
     * The robot's shell ID. Used for debug drawing.
     */
    unsigned shell_id{};

    /**
     * The state of the world, containing robot and ball states.
     *
     * For obstacle-avoidance purposes, obstacles should be used instead. This
     * can be used for lookup of robots/balls by ID.
     */
    WorldState world_state{};

    /**
     * The state of the game and referee
     */
    PlayState play_state;

    /**
     * The priority of this plan request.
     */
    int8_t priority = 0;

    /**
     * Allows debug drawing in the world. If this is nullptr, no debug drawing
     * should be performed.
     */
    rj_drawing::RosDebugDrawer* debug_drawer = nullptr;

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

    /**
     * Store the ball's trajectory if we want to avoid it.
     */
    Trajectory ball_trajectory{};

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
static rj_geometry::Circle create_robot_obstacle(const RobotState& robot);
}  // namespace planning
