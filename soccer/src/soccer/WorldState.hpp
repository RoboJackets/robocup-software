#pragma once

#include <Geometry2d/Pose.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/detail/ball_state__builder.hpp>
#include <rj_msgs/msg/detail/robot_state__builder.hpp>
#include <rj_msgs/msg/detail/world_state__builder.hpp>

#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"

// TODO(#1498): Make this configurable
constexpr double kBallDecayConstant = 0.180;

/**
 * @brief Contains robot motion state data
 * @details This class contains data that comes from the vision system
 * including position data and which camera this robot was seen by and
 * what time it was last seen.
 */
struct RobotState {
    using Msg = rj_msgs::msg::RobotState;

    Geometry2d::Pose pose;
    Geometry2d::Twist velocity;
    RJ::Time timestamp;
    bool visible = false;

    /**
     * @brief Default constructor: an invalid robot with visible=false.
     */
    RobotState() = default;

    RobotState(const Geometry2d::Pose& pose, const Geometry2d::Twist& velocity,
               const RJ::Time& timestamp, bool visible)
        : pose{pose},
          velocity{velocity},
          timestamp{timestamp},
          visible{visible} {}

    /**
     * @brief Implicit conversion from RobotState::Msg.
     * @param msg
     */
    RobotState(const Msg& msg)
        : RobotState{msg.pose, msg.velocity, RJ::FromROSTime(msg.stamp),
                     msg.visible} {}

    /**
     * @brief Implicit conversion to RobotState::Msg.
     * @return
     */
    [[nodiscard]] operator Msg() const {
        return rj_msgs::build<Msg>()
            .stamp(RJ::ToROSTime(timestamp))
            .pose(pose)
            .velocity(velocity)
            .visible(visible);
    }
};

/**
 * @brief Our belief about the ball's current position and velocity.
 */
struct BallState {
    using Msg = rj_msgs::msg::BallState;

    Geometry2d::Point position;
    Geometry2d::Point velocity;
    RJ::Time timestamp;
    bool visible = false;

    /**
     * @brief Default constructor: an invalid ball with visible=false.
     */
    BallState() = default;

    /**
     * @brief Construct a BallState with a valid estimate.
     */
    BallState(Geometry2d::Point position, Geometry2d::Point velocity,
              RJ::Time timestamp = RJ::now())
        : position(position), velocity(velocity), timestamp(timestamp) {
        visible = true;
    }

    /**
     * @brief Implicit conversion from BallState::Msg.
     * @param msg
     */
    BallState(const Msg& msg)
        : BallState{msg.position, msg.velocity, RJ::FromROSTime(msg.stamp)} {}

    /**
     * @brief Predict the ball's state at a particular instance in time.
     *
     * @param Time the time at which to evaluate the ball's position.
     * @return The ball's state.
     */
    [[nodiscard]] BallState predict_at(RJ::Time time) const;

    /**
     * @brief Similar to @ref predict_at "predict_at(RJ::Time)", but for a
     * duration in the future (offset calculated from the ball's sample time).
     *
     * @param seconds The offset from this ball's sample time at which to
     * predict its motion.
     * @return The ball's motion at the specified duration after this sample.
     */
    [[nodiscard]] BallState predict_in(RJ::Seconds seconds) const;

    /**
     * @brief Estimate the instant in time at which the ball will reach the
     * given position (or the nearest point along the line of its path).
     *
     * @detail If the ball will never reach the nearest point along its line,
     * return the ball's endpoint and time.
     *
     * @param near_to The query point
     * @param out The nearest point to `near_to` along the path.
     * @return The instant in time at which the ball is nearest to `near_to`.
     */
    [[nodiscard]] RJ::Time query_time_near(
        Geometry2d::Point near_to, Geometry2d::Point* out = nullptr) const;

    /**
     * @brief Similar to @ref predict_at "query_time_near(RJ::Time)", but for a
     * duration in the future
     */
    [[nodiscard]] RJ::Seconds query_seconds_near(
        Geometry2d::Point near_to, Geometry2d::Point* out = nullptr) const;

    /**
     * @brief Predict the stop time of the ball.
     *
     * @param out will be filled with the stopping position, if it is not
     * nullptr.
     * @return The duration until the ball stops.
     */
    [[nodiscard]] RJ::Seconds query_stop_time(
        Geometry2d::Point* out = nullptr) const;

    /**
     * @brief Predict the stop position of the ball.
     *
     * @return The point at which the ball will stop.
     */
    [[nodiscard]] Geometry2d::Point query_stop_position() const;

    /**
     * @brief Query the time before the ball goes a certain distance. Return
     * nullopt if it will stop before traveling the specified distance.
     *
     * @param distance The distance along the ball path at which to query.
     * @return The time at which the ball will hit the specified distance, or
     * nullopt.
     */
    [[nodiscard]] std::optional<RJ::Seconds> query_seconds_to_dist(
        double distance) const;

    /**
     * @brief Create a trajectory for the ball.
     * @return A trajectory for this ball to follow. Angles are meaningless.
     */
    [[nodiscard]] Planning::Trajectory make_trajectory() const;
};

struct WorldState {
    using Msg = rj_msgs::msg::WorldState;

    WorldState() {
        their_robots.resize(Num_Shells);
        our_robots.resize(Num_Shells);
    }

    /**
     * @brief Implicit conversion from WorldState::Msg.
     */
    WorldState(const Msg& msg)
        : their_robots(msg.their_robots.begin(), msg.their_robots.end()),
          our_robots(msg.our_robots.begin(), msg.our_robots.end()),
          ball{msg.ball} {}

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
