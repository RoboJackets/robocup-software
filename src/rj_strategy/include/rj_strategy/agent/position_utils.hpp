#pragma once

#include <cstdlib>
#include <cmath>
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
 * Rules:
 *  - these functions have to be stateless and context-free
 *      yes it's in namespace strategy, but that's only to make sure others can see it
 *  - try to keep big object args as const references (to minimize overhead)
 *  - try to keep the execution time minimal (to minimize overhead)
 *  - try to keep function names unnecessarily verbose and accurate (to minimize headache)
 *  - currently, I am not worried about bloat in this file
 *      if you add util, add it here, not in your strategy cpp
 *  - if you love me, do not use auto. figure out what type you want
 */

namespace strategy {

// Example
/**
 * @return the Euclidean distance between the points 
 */
inline double distance(const rj_geometry::Point& a, const rj_geometry::Point& b) {
    return (a - b).mag();
}


// Geometry confirmation
/**
 * @return whether the ball is on the play field
 */
inline bool ball_on_field(const WorldState* world_state, const FieldDimensions& field_dimensions) {
    const rj_geometry::Point& ball_point = world_state->ball.position;
    return field_dimensions.field_rect().contains_point(ball_point);
}

/**
 * @return whether the ball is in our goalie box
 */
inline bool ball_in_our_goalie_box(const WorldState* world_state, const FieldDimensions& field_dimensions) {
    const rj_geometry::Point& ball_point = world_state->ball.position;
    return field_dimensions.our_defense_area().contains_point(ball_point);
}

/**
 * @return whether the ball is on the play field
 */
inline bool ball_in_their_goalie_box(const WorldState* world_state, const FieldDimensions& field_dimensions) {
    const rj_geometry::Point& ball_point = world_state->ball.position;
    return field_dimensions.their_defense_area().contains_point(ball_point);
}

/**
 * @return whether the ball is in an area that non-goalies cannot reach.
 */
inline bool ball_in_play_area(const WorldState* world_state, const FieldDimensions& field_dimensions) {
    return ball_on_field(world_state, field_dimensions) &&
           !ball_in_our_goalie_box(world_state, field_dimensions) &&
           !ball_in_their_goalie_box(world_state, field_dimensions);
}


// Shot calculation
/**
 * @return the smallest angular distance from a proposed shot to opponents
 * 0 means fully covered, pi/2 means clear shot
 * TODO: I don't think our robots (any position) are programmed to get out of the way of shots, frankly, we may need to consider them opponents
 */
inline double shot_clearance(const rj_geometry::Point& origin, const rj_geometry::Point& shot, const WorldState* world_state) {
    rj_geometry::Point shot_vec = shot - origin;
    const auto& their_robots = world_state->their_robots; // TODO: no full auto in geometry_utils.hpp, what type is this?
    double min_angle = M_PI_2;
    for (const auto& enemy : their_robots) {
        rj_geometry::Point enemy_vec = enemy.pose.position() - origin;
        if (enemy_vec.dot(shot_vec) < 0) { // if enemy is behind us,
            continue; // ignore enemy
        }

        double projection = enemy_vec.dot(shot_vec) / shot_vec.dot(shot_vec);
        double dist_to_block = (enemy_vec - projection * shot_vec).mag(); // magnitude of the perpendicular component
        dist_to_block = std::max(0.0, dist_to_block - kRobotRadius - kBallRadius); // (robot positions are the center, but even glancing a shot will usually block)
        double dist_along_shot = projection * shot_vec.mag(); // magnitude of the tangent component

        double clearance_angle = std::atan2(dist_to_block, dist_along_shot);
        min_angle = std::min(min_angle, clearance_angle);
    }
    return min_angle;
}

/**
 * @return a good shot worth taking, given our practical realities
 * currently just returns the center of the goal
 * TODO: can we quantify this with angular width, and parametrize confidence?
 */
inline rj_geometry::Point calculate_a_shot([[maybe_unused]] const WorldState* world_state, const FieldDimensions& field_dimensions) {
    return field_dimensions.their_goal_loc();
}


/**
 * @return the best theoretically available shot, assuming a very precise shooter
 * TODO: even if we have infinite precision, it would be inappropriate to shoot at the posts; clearance zone parametrized by ball radius?
 */
inline rj_geometry::Point calculate_bestest_shot(const WorldState* world_state, const FieldDimensions& field_dimensions, double granularity) {
    // Geometry
    rj_geometry::Point their_goal_pos = field_dimensions.their_goal_loc();
    double goal_width = field_dimensions.goal_width();
    rj_geometry::Point left_post = their_goal_pos - rj_geometry::Point(goal_width / 2.0, 0.0);
    rj_geometry::Point ball_pos = world_state->ball.position;
    // We're argmaxxing
    rj_geometry::Point best_shot = left_post;
    double best_clearance = -1.0;
    // Iteration over the goal space at granularity
    int num_iters = static_cast<int>(goal_width / granularity);
    rj_geometry::Point increment(granularity, 0);
    rj_geometry::Point curr_shot;
    for (int i = 0; i < num_iters; i++) {
        curr_shot = left_post + increment * i;
        double curr_clearance = shot_clearance(ball_pos, curr_shot, world_state);
        if (curr_clearance > best_clearance) {
            best_shot = curr_shot;
            best_clearance = curr_clearance;
        }
    }

    return best_shot;
}

} // namespace strategy