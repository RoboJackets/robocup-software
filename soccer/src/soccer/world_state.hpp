#pragma once

#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/ball_state.hpp>
#include <rj_msgs/msg/robot_state.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <global_params.hpp>

#include "planning/instant.hpp"
#include "planning/trajectory.hpp"

/**
 * @brief Contains robot motion state data
 * @details This class contains data that comes from the vision system
 * including position data and which camera this robot was seen by and
 * what time it was last seen.
 */
struct RobotState {
    using Msg = rj_msgs::msg::RobotState;

    rj_geometry::Pose pose;
    rj_geometry::Twist velocity;
    RJ::Time timestamp;
    bool visible = false;

    /**
     * @brief Default constructor: an invalid robot with visible=false.
     */
    RobotState() = default;

    RobotState(const rj_geometry::Pose& pose, const rj_geometry::Twist& velocity,
               const RJ::Time& timestamp, bool visible)
        : pose{pose},
          velocity{velocity},
          timestamp{timestamp},
          visible{visible} {}
};

/**
 * @brief Our belief about the ball's current position and velocity.
 */
struct BallState {
    using Msg = rj_msgs::msg::BallState;

    rj_geometry::Point position;
    rj_geometry::Point velocity;
    RJ::Time timestamp;
    bool visible = false;

    /**
     * @brief Default constructor: an invalid ball with visible=false.
     */
    BallState() = default;

    /**
     * @brief Construct a BallState with a valid estimate.
     */
    BallState(rj_geometry::Point position, rj_geometry::Point velocity,
              RJ::Time timestamp = RJ::now())
        : position(position), velocity(velocity), timestamp(timestamp) {
        visible = true;
    }

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
        rj_geometry::Point near_to, rj_geometry::Point* out = nullptr) const;

    /**
     * @brief Similar to @ref predict_at "query_time_near(RJ::Time)", but for a
     * duration in the future
     */
    [[nodiscard]] RJ::Seconds query_seconds_near(
        rj_geometry::Point near_to, rj_geometry::Point* out = nullptr) const;

    /**
     * @brief Predict the stop time of the ball.
     *
     * @param out will be filled with the stopping position, if it is not
     * nullptr.
     * @return The duration until the ball stops.
     */
    [[nodiscard]] RJ::Seconds query_stop_time(
        rj_geometry::Point* out = nullptr) const;

    /**
     * @brief Predict the stop position of the ball.
     *
     * @return The point at which the ball will stop.
     */
    [[nodiscard]] rj_geometry::Point query_stop_position() const;

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
    [[nodiscard]] planning::Trajectory make_trajectory() const;
};

struct WorldState {
    using Msg = rj_msgs::msg::WorldState;

    WorldState() {
        their_robots.resize(kNumShells);
        our_robots.resize(kNumShells);
    }

    /**
     * @brief Constructor for WorldState.
     */
    WorldState(std::vector<RobotState> their_robots,
               std::vector<RobotState> our_robots, const BallState& ball)
        : their_robots{std::move(their_robots)},
          our_robots{std::move(our_robots)},
          ball{ball} {}

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

    /**
     * @brief Timestamp of the last received vision message. All zeros if we
     * haven't received anything yet.
     */
    RJ::Time last_updated_time;

    std::vector<RobotState> their_robots;
    std::vector<RobotState> our_robots;
    BallState ball;
};

namespace rj_convert {

template <>
struct RosConverter<RobotState, RobotState::Msg> {
    static RobotState::Msg to_ros(const RobotState& value) {
        RobotState::Msg result;
        convert_to_ros(value.timestamp, &result.stamp);
        convert_to_ros(value.pose, &result.pose);
        convert_to_ros(value.velocity, &result.velocity);
        convert_to_ros(value.visible, &result.visible);
        return result;
    }

    static RobotState from_ros(const RobotState::Msg& value) {
        RobotState result;
        result.timestamp = convert_from_ros(value.stamp);
        convert_from_ros(value.pose, &result.pose);
        convert_from_ros(value.velocity, &result.velocity);
        convert_from_ros(value.visible, &result.visible);
        return result;
    }
};

ASSOCIATE_CPP_ROS(RobotState, RobotState::Msg);

template <>
struct RosConverter<BallState, BallState::Msg> {
    static BallState::Msg to_ros(const BallState& value) {
        BallState::Msg result;
        convert_to_ros(value.timestamp, &result.stamp);
        convert_to_ros(value.velocity, &result.velocity);
        convert_to_ros(value.position, &result.position);
        convert_to_ros(value.visible, &result.visible);
        return result;
    }

    static BallState from_ros(const BallState::Msg& value) {
        BallState result;
        convert_from_ros(value.stamp, &result.timestamp);
        convert_from_ros(value.velocity, &result.velocity);
        convert_from_ros(value.position, &result.position);
        convert_to_ros(value.visible, &result.visible);
        return result;
    }
};

ASSOCIATE_CPP_ROS(BallState, BallState::Msg);

template <>
struct RosConverter<WorldState, WorldState::Msg> {
    static WorldState::Msg to_ros(const WorldState& value) {
        WorldState::Msg result;
        convert_to_ros(value.ball, &result.ball);
        convert_to_ros(value.our_robots, &result.our_robots);
        convert_to_ros(value.their_robots, &result.their_robots);
        convert_to_ros(value.last_updated_time, &result.last_update_time);
        return result;
    }

    static WorldState from_ros(const WorldState::Msg& value) {
        WorldState result;
        convert_from_ros(value.ball, &result.ball);
        convert_from_ros(value.our_robots, &result.our_robots);
        convert_from_ros(value.their_robots, &result.their_robots);
        convert_from_ros(value.last_update_time, &result.last_updated_time);
        return result;
    }
};

ASSOCIATE_CPP_ROS(WorldState, WorldState::Msg);

}  // namespace rj_convert
