#pragma once

#include <Constants.hpp>
#include <Geometry2d/Pose.hpp>
#include <planning/trajectory/Trajectory.hpp>

#include "time.hpp"

// TODO: Make this configurable
constexpr double kBallDecayConstant = 0.180;

/**
 * @brief Contains robot motion state data
 * @details This class contains data that comes from the vision system
 * including position data and which camera this robot was seen by and
 * what time it was last seen.
 */
struct RobotState {
    Geometry2d::Pose pose;
    Geometry2d::Twist velocity;
    RJ::Time timestamp;
    bool visible = false;
    bool velocity_valid = false;
};

/**
 * Our belief about the ball's current position and velocity.
 */
struct BallState {
    Geometry2d::Point position;
    Geometry2d::Point velocity;
    RJ::Time timestamp;
    bool visible = false;

    /**
     * Default constructor: an invalid ball with visible=false.
     */
    BallState() = default;

    /**
     * Construct a BallState with a valid estimate.
     */
    BallState(Geometry2d::Point position,
              Geometry2d::Point velocity,
              RJ::Time timestamp = RJ::now())
        : position(position),
          velocity(velocity),
          timestamp(timestamp) {
        visible = true;
    }

    /**
     * Predict the ball's state at a particular instance in time.
     * @param time the time at which to evaluate the ball's position.
     * @return the ball's state.
     */
    [[nodiscard]] BallState predict_at(RJ::Time time) const;

    /**
     * Similar to \ref predict_at "predict_at(RJ::Time)", but for a duration in
     * the future
     */
    [[nodiscard]] BallState predict_in(RJ::Seconds seconds) const;

    /**
     * Estimate the instant in time at which the ball will reach the given
     * position (or the nearest point along its path to this position).
     * @param position the query point
     * @param out the nearest point to `position` along the path.
     * @return the instant in time at which the ball is nearest to `position`.
     */
    [[nodiscard]] RJ::Time query_time_at(
        Geometry2d::Point position,
        Geometry2d::Point* out = nullptr) const;

    /**
     * Similar to \ref predict_at "query_time_at(RJ::Time)", but for a duration
     * in the future
     */
    [[nodiscard]] RJ::Seconds query_seconds_to(
        Geometry2d::Point position,
        Geometry2d::Point* out = nullptr) const;

    /**
     * Predict the stop time of the ball.
     * @return the duration until the ball stops.
     */
    [[nodiscard]] RJ::Seconds query_stop_time() const;

    /**
     * Query the time before the ball goes a certain distance.
     */
    [[nodiscard]] std::optional<RJ::Seconds> query_seconds_to_dist(double distance) const;

    /**
     * Create a trajectory for the ball.
     */
    [[nodiscard]] Planning::Trajectory make_trajectory();
};

struct WorldState {
    WorldState() {
        their_robots.resize(Num_Shells);
        our_robots.resize(Num_Shells);
    }

    RobotState& get_robot(bool ours, int shell) {
        if (ours) {
            return our_robots.at(shell);
        } else {
            return their_robots.at(shell);
        }
    }

    [[nodiscard]] RobotState get_robot(bool ours, int shell) const {
        if (ours) {
            return our_robots.at(shell);
        } else {
            return their_robots.at(shell);
        }
    }

    std::vector<RobotState> their_robots;
    std::vector<RobotState> our_robots;
    BallState ball;
};
