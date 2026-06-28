#pragma once

#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/time.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>

/**
 * This file is just to collect a bunch of common utilities.
 * Rules:
 *  - These functions should be stateless and context-free.
 *  - Try to keep big object args as const references (to minimize overhead).
 *  - Try to keep the execution time minimal (to minimize overhead).
 *  - Try to keep function names unnecessarily verbose and accurate (to minimize headache).
 *  - Remember that if you change something, other people may be using that; don't fundamentally
 *    change invariants. If you're doing something a little different, do it in your own file,
 *    or add a new util, avoid revising where possible.
 *  - Please don't use auto. Figure out what type you want :)
 * Longterm TODO:
 *  - A lot functions in here do pairwise distance between various objects. Can we have a
 *    coordinator that does a grand pairwise distance we can just reference?
 *  - defense area vs penalty area, more in-depth discussion below
 */

// Field geometry interfacing
/**
 * @brief Determines whether the ball is in bounds (whole field rectangle).
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param field_dimensions (often named field_dimensions_ in Position subclasses)
 * @return if ball.position is in bounds
 */
inline bool ball_on_field(const WorldState* world_state, const FieldDimensions& field_dimensions) {
    const rj_geometry::Point& ball_point = world_state->ball.position;
    return field_dimensions.field_rect().contains_point(ball_point);
}

/** BIG TODO
 * The below functions refer to a "defense area". This is not what the rules refer to as
 * "defense area". In the rules, "defense area" means the white lines of the goalie box.
 * We have a widened red region that should properly render in sim. I believe this was
 * done for the sake of rules compliance, so we don't accidentally commit violations upon
 * overshoots. In the future, we should consider how to play tussles at the border of the
 * defense area, but we should use the widened red box.
 *
 * If you really want to use the goalie lines, we call this "penalty area" in
 * FieldDimensions. I won't expose it here.
 */

/**
 * @brief Determines whether the ball is in our defense area (red region around goalie box).
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param field_dimensions (often named field_dimensions_ in Position subclasses)
 * @return if ball.position is in our goalie box
 */
inline bool ball_in_our_defense_area(const WorldState* world_state,
                                     const FieldDimensions& field_dimensions) {
    const rj_geometry::Point& ball_point = world_state->ball.position;
    return field_dimensions.our_defense_area().contains_point(ball_point);
}

/**
 * @brief Determines whether the ball is in our defense area (red region around goalie box).
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param field_dimensions (often named field_dimensions_ in Position subclasses)
 * @return if ball.position is in their goalie box
 */
inline bool ball_in_their_defense_area(const WorldState* world_state,
                                       const FieldDimensions& field_dimensions) {
    const rj_geometry::Point& ball_point = world_state->ball.position;
    return field_dimensions.their_defense_area().contains_point(ball_point);
}

/**
 * @return Determines whether the ball is in an area that non-goalies can reach.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param field_dimensions (often named field_dimensions_ in Position subclasses)
 * @return if ball.position is in playable space
 */
inline bool ball_in_play_area(const WorldState* world_state,
                              const FieldDimensions& field_dimensions) {
    return ball_on_field(world_state, field_dimensions) &&
           !ball_in_our_defense_area(world_state, field_dimensions) &&
           !ball_in_their_defense_area(world_state, field_dimensions);
}

// Shot calculation
/**
 * @brief Gets the shortest angular distance from a proposed shotline to an opponent.
 * Can be thought of as half of the the arc of the largest circular sector centered around the shot
 * that has no opponent bits in it.
 *
 * The idea is that we want to take shots that are far away from opponents.
 * Currently we pick the one that is furthest angularly from an opponent, to minimize the chance of
 * them blocking. 0 means that there is an opponent directly on the shotline. pi/2 means that there
 * is no opponent bot in the forward FOV.
 *
 * @param origin where the ball starts
 * @param shot where the shot is aimed
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @return the size of the smallest angle to defender
 */
inline double shot_clearance(const rj_geometry::Point& origin, const rj_geometry::Point& shot,
                             const WorldState* world_state) {
    rj_geometry::Point shot_vec = shot - origin;
    const std::vector<RobotState>& their_robots = world_state->their_robots;
    double min_angle = M_PI_2;
    for (const RobotState& enemy :
         their_robots) {  // TODO: our robots are not programmed to dodge our own shots, frankly, we
                          // may need to consider them opponents
        rj_geometry::Point enemy_vec = enemy.pose.position() - origin;
        if (enemy_vec.dot(shot_vec) < 0) {
            continue;  // if the enemy is behind us, ignore them
        }

        double projection = enemy_vec.dot(shot_vec) / shot_vec.dot(shot_vec);
        double dist_to_block =
            (enemy_vec - projection * shot_vec).mag();  // magnitude of the perpendicular component
        dist_to_block = std::max(0.0, dist_to_block - kRobotRadius -
                                          kBallRadius);  // (robot positions are the center, but
                                                         // even glancing a shot will usually block)
        double dist_along_shot = projection * shot_vec.mag();  // magnitude of the tangent component

        double clearance_angle = std::atan2(dist_to_block, dist_along_shot);
        min_angle = std::min(min_angle, clearance_angle);
    }
    return min_angle;
}

/**
 * @brief Gets a good shot worth taking, given our pratical realities.
 * Use this function and rest assured it will go in on an undefended goal.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param field_dimensions (often named field_dimensions_ in Position subclasses)
 * @return a point to aim at for the good shot
 *   * currently just returns the center of the goal
 */
inline rj_geometry::Point calculate_a_shot([[maybe_unused]] const WorldState* world_state,
                                           const FieldDimensions& field_dimensions) {
    // TODO: could be a fun collab with hardware!
    // Can we quantify our uncertainty and aim closer to the goalposts when close enough?
    return field_dimensions.their_goal_loc();
}

/**
 * @brief Gets a the best shot available, given excellent aim.
 * The definition of "best" may need refining, but I think my approach is sound.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param field_dimensions (often named field_dimensions_ in Position subclasses)
 * @param granularity [OPTIONAL] the spacing between considered points on the goal line (meters)
 * [default: 0.04 (a constant)]
 * @param ignore_posts [OPTIONAL] whether to count the goalposts as candidate shots [default: true]
 * @return a point to aim at for the best shot
 */
inline rj_geometry::Point calculate_best_shot(const WorldState* world_state,
                                              const FieldDimensions& field_dimensions,
                                              double granularity = kShotCalculationGranularity,
                                              bool ignore_posts = true) {
    // An initial guess can be center of goal.
    const rj_geometry::Point& enemy_goal_center = field_dimensions.their_goal_loc();
    if (granularity <= 0.0) {
        SPDLOG_ERROR("granularity arg passed into calculate_best_shot must be positive");
        return enemy_goal_center;
    }

    // Make a better choice by scanning the goal.
    // Scan linearly over x positions in the goal from post to post.
    const double goal_width = field_dimensions.goal_width();
    rj_geometry::Point lower_bound = enemy_goal_center - rj_geometry::Point(goal_width / 2.0, 0.0);
    rj_geometry::Point upper_bound = enemy_goal_center + rj_geometry::Point(goal_width / 2.0, 0.0);
    rj_geometry::Point increment(granularity, 0.0);
    if (ignore_posts) {
        lower_bound = lower_bound + increment;
        upper_bound = upper_bound - increment;
    }
    rj_geometry::Point ball_pos = world_state->ball.position;

    // Argmaxxing over all scan points.
    rj_geometry::Point best_shot(0.0, 0.0);
    double best_clearance = -1.0;
    rj_geometry::Point curr_shot = lower_bound;
    double curr_clearance;
    while (curr_shot.x() < upper_bound.x()) {
        curr_clearance = shot_clearance(ball_pos, curr_shot, world_state);
        if (curr_clearance > best_clearance) {
            best_shot = curr_shot;
            best_clearance = curr_clearance;
        }
        curr_shot = curr_shot + increment;
    }

    // Increment may not align with the upperbound, so do a final manual check.
    curr_clearance = shot_clearance(ball_pos, upper_bound, world_state);
    if (curr_clearance > best_clearance) {
        best_shot = upper_bound;
    }

    return best_shot;
}

// Possession calculation
/**
 * @brief Determines whether a specific robot has possession of the ball.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param possession_radius [OPTIONAL] the distance at which a robot is defined to "have" the ball
 * (m) [default: kRobotRadius]
 * @return does it have ball
 */
inline bool robot_has_ball(const WorldState* world_state, const RobotState& robot,
                           double possession_radius = 2 * kRobotRadius) {
    // TODO: this function should probably account for rotation
    //       a robot cannot take dribble possession with its rear wheels

    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point robot_pos = robot.pose.position();
    return ball_pos.dist_to(robot_pos) < possession_radius;
}

/**
 * @brief Determines whether the enemy has possession of the ball.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param possession_radius [OPTIONAL] the distance at which a robot is defined to "have" the ball
 * (m) [default: kRobotRadius]
 * @return do they have ball
 */
inline bool they_have_ball(const WorldState* world_state, double possession_radius = kRobotRadius) {
    const std::vector<RobotState>& theirs = world_state->their_robots;
    for (const RobotState& opponent : theirs) {
        if (robot_has_ball(world_state, opponent, possession_radius)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Determines whether our team has possession of the ball.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param possession_radius [OPTIONAL] the distance at which a robot is defined to "have" the ball
 * (m) [default: kRobotRadius]
 * @return do we have ball
 */
inline bool we_have_ball(const WorldState* world_state, double possession_radius = kRobotRadius) {
    const std::vector<RobotState>& ours = world_state->our_robots;
    for (const RobotState& teammate : ours) {
        if (robot_has_ball(world_state, teammate, possession_radius)) {
            return true;
        }
    }
    return false;
}

// Kick speed calculation
/**
 * @brief Provides a good suggestion for kick speed, designed for passing.
 *
 * Motion intents take an integer from [0,15]. This provides that.
 *
 * @param distance_to_target distance from kicker to target (m)
 * @param intended_velo_at_target ideal ball velocity at the target (m/s)
 * @return an int to shove in the motion command
 */
inline int calculate_kick_speed(double distance_to_target, double intended_velo_at_target) {
    // Without measurement of anything, we cannot make a cogent estimate of kick speed.
    if (distance_to_target < 0.6) {
        return 5;
    } else if (distance_to_target < 1.8) {
        return 6;
    }

    return 7;

    // TODO: these numbers are imaginary; based on estimates, we NEED to measure
    // This is the approach we should take to kick speed, do not delete this.
    // This could be a fun collab with the hardware subteam! They could build a light gate!

    // This maps "kick power" to the initial velocity of the ball (m/s).
    constexpr std::array<double, 16> kick_speed_map = {3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0,  6.5,
                                                       7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0, 10.5};
    // This notes the deceleration of the ball on the pitch (m/s^2).
    const double ball_deceleration = 1.75;  // ideally, this is parametrized in rqt

    // Compute ideal velo (v_final^2 = v_initial^2 - 2ad).
    double target_velocity = std::sqrt(std::pow(intended_velo_at_target, 2.0) +
                                       2 * ball_deceleration * distance_to_target);
    if (target_velocity > 6.5) {  // rules tolerance
        target_velocity = 6.5;
    }

    // Return the first candidate that exceeds the needed velocity (err on faster passes).
    for (int i = 0; i < 16; i++) {
        if (kick_speed_map[i] > target_velocity) {
            return i;
        }
    }
    return 15;
}

/**
 * @brief Provides the max allowed kick speed, designed for shooting.
 *
 * Motion intents take an integer from [0,15]. This provides that.
 *
 * The max ball speed is 6.5m/s. In Brasil25, using 7 was generally safe.
 *
 * @return 7
 */
inline int max_kick_speed() { return 7; }
