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
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>

/**
 * This file is just to collect a bunch of common utilities.
 * Longterm TODO:
 *  - a lot functions in here do pairwise distance between various objects. can we have a
 *    coordinator that does a grand pairwise distance we can just reference?
 * Rules:
 *  - these functions have to be stateless and context-free
 *  - try to keep big object args as const references (to minimize overhead)
 *  - try to keep the execution time minimal (to minimize overhead)
 *  - try to keep function names unnecessarily verbose and accurate (to minimize headache)
 *  - remember that if you change something, other people may be using that; don't fundamentally
 *    change invariants if you're doing something a little different, do it in your own file,
 *    or add a new util, avoid revising where possible
 *  - please don't use auto. figure out what type you want :)
 */

// namespace strategy {

// Example
/**
 * @brief Euclidean distance between two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return the Euclidean distance between a and b
 */
inline double distance(const rj_geometry::Point& a, const rj_geometry::Point& b) {
    return (a - b).mag();
}

// Geometry confirmation
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

// There is also "penalty area". This refers to the actual white lines of the goalie box.
// Annoyingly, this is what is called "defense area" in the rules document.
// I have chosen not to expose this, since we have a widened red region around the box lines for
// rules compliance. We're not yet at the skill level to bother with tussles at the defense lines.

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
 * @param granularity the spacing between considered points on the goal line (meters)
 * @param ignore_posts whether to count the goalposts as candidate shots (you should use True,
 * aiming for the goalposts is stupid)
 * @return a point to aim at for the best shot
 */
inline rj_geometry::Point calculate_best_shot(const WorldState* world_state,
                                              const FieldDimensions& field_dimensions,
                                              double granularity, bool ignore_posts) {
    // Geometry
    rj_geometry::Point their_goal_pos =
        field_dimensions.their_goal_loc();  // returns center of goal
    if (granularity <= 0.0) {               // protection
        SPDLOG_ERROR(
            "Invalid granularity value passed into calculate_best_shot, must use a positive "
            "float.");
        return their_goal_pos;
    }
    double goal_width = field_dimensions.goal_width();
    rj_geometry::Point negXmost_shot = their_goal_pos - rj_geometry::Point(goal_width / 2.0, 0.0);
    rj_geometry::Point posXmost_shot = their_goal_pos + rj_geometry::Point(goal_width / 2.0, 0.0);
    rj_geometry::Point increment(granularity, 0.0);
    if (ignore_posts) {
        negXmost_shot = negXmost_shot + increment;
        posXmost_shot = posXmost_shot - increment;
    }
    rj_geometry::Point ball_pos = world_state->ball.position;

    // Argmaxxing
    rj_geometry::Point best_shot(0.0, 0.0);
    double curr_clearance;
    double best_clearance =
        -1.0;  // clearance is in [0,pi/2], so this will be immediately overwritten

    rj_geometry::Point curr_shot = negXmost_shot;
    while (curr_shot.x() <= posXmost_shot.x()) {
        curr_clearance = shot_clearance(ball_pos, curr_shot, world_state);
        if (curr_clearance > best_clearance) {
            best_shot = curr_shot;
            best_clearance = curr_clearance;
        }
        curr_shot = curr_shot + increment;
    }
    // Increment may not align with the posX side exactly, so do a manual check.
    curr_clearance = shot_clearance(ball_pos, posXmost_shot, world_state);
    if (curr_clearance > best_clearance) {
        best_shot = posXmost_shot;
    }

    return best_shot;
}

// Possession calculation
// TODO: these functions should also account for rotation. a robot doesn't have possession if the
// ball is sitting at its rear motors
// TODO: overload these functions so that we can have some default possession_radius, such that
// people using it don't need to turn on they brain
/**
 * @brief Determines whether the enemy has possession of the ball.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param possession_radius the distance at which a robot is defined to "have" the ball (m)
 * @return do they have ball
 */
inline bool they_have_ball(const WorldState* world_state, double possession_radius) {
    const std::vector<RobotState>& theirs = world_state->their_robots;
    rj_geometry::Point ball_pos = world_state->ball.position;
    for (const RobotState& enemy : theirs) {
        rj_geometry::Point enemy_pos = enemy.pose.position();
        if (distance(ball_pos, enemy_pos) < possession_radius) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Determines whether our team has possession of the ball.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param possession_radius the distance at which a robot is defined to "have" the ball (m)
 * @return do we have ball
 */
inline bool we_have_ball(const WorldState* world_state, double possession_radius) {
    const std::vector<RobotState>& ours = world_state->our_robots;
    rj_geometry::Point ball_pos = world_state->ball.position;
    for (const RobotState& teammate : ours) {
        rj_geometry::Point teammate_pos = teammate.pose.position();
        if (distance(ball_pos, teammate_pos) < possession_radius) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Determines whether a specific robot has possession of the ball.
 *
 * @param world_state (often named last_world_state_ in Position subclasses)
 * @param possession_radius the distance at which a robot is defined to "have" the ball (m)
 * @return does it have ball
 */
inline bool robot_has_ball(const WorldState* world_state, const RobotState& robot,
                           double possession_radius) {
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point robot_pos = robot.pose.position();
    if (distance(ball_pos, robot_pos) < possession_radius) {
        return true;
    } else {
        return false;
    }
}

// Kick speed calculation
/**
 * @brief Provides a good suggestion for kick speed, designed for passing.
 *
 * Motion intents take an integer from [0,15]. This provides that.
 *
 * @param distance_to_target distance from kicker to target (m)
 * @param intended_velo_at_target the velocity you want the ball to be going when it reaches the
 * target (m/s)
 * @return an int to shove in the motion command
 */
inline int calculate_kick_speed(double distance_to_target,
                                [[maybe_unused]] double intended_velo_at_target) {
    // TODO: this could be a fun collab with hardware!
    // v^2 = v_initial^2 - 2ad, where a is a measure of the constant deceleration applied from the
    // ground Frankly, that changes from pitch to pitch, so it should probably be parametrized in
    // rqt. We would also then need a map from v_initial to power[0,15]

    // The below code is very dumb. I stole it from Offense.
    if (distance_to_target < 0.6) {
        return 3;
    } else if (distance_to_target < 1.8) {
        return 4;
    } else {
        return 5;
    }
}

/**
 * @brief Provides the max allowed kick speed, designed for shooting.
 *
 * Motion intents take an integer from [0,15]. This provides that.
 *
 * @return 7
 */
inline int max_kick_speed() {
    // TODO: obviously this needs to be tested, 6.5m/s is the rules limit
    return 7;
}

// } // namespace strategy