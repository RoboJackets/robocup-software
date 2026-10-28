#pragma once

#include <memory>

#include <rj_common/time.hpp>

#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/ball_state.hpp>
#include <rj_msgs/msg/robot_state.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/global_params.hpp>

#include "rj_common/game_constants.hpp"
#include "rj_common/planning/instant.hpp"
#include "rj_common/planning/trajectory.hpp"

/**
 * @brief Contains robot motion state data
 * @details This class contains data that comes from the vision system
 * including position data and which camera this robot was seen by and
 * what time it was last seen.
 */
struct RobotState {
    using Msg = rj_msgs::msg::RobotState;
    using SharedPtr = std::shared_ptr<RobotState>;
    using ConstSharedPtr = std::shared_ptr<const RobotState>;

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
    using SharedPtr = std::shared_ptr<BallState>;
    using ConstSharedPtr = std::shared_ptr<const BallState>;

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
    using SharedPtr = std::shared_ptr<WorldState>;
    using ConstSharedPtr = std::shared_ptr<const WorldState>;

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

namespace rclcpp {

template <>
struct TypeAdapter<RobotState, RobotState::Msg> {
    using is_specialized = std::true_type;
    using custom_type = RobotState;
    using ros_message_type = RobotState::Msg;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rclcpp::TypeAdapter<RJ::Time, builtin_interfaces::msg::Time>::convert_to_ros_message(
            source.timestamp, destination.stamp);
        rclcpp::TypeAdapter<rj_geometry::Pose, rj_geometry_msgs::msg::Pose>::convert_to_ros_message(
            source.pose, destination.pose);
        rclcpp::TypeAdapter<rj_geometry::Twist, rj_geometry_msgs::msg::Twist>::convert_to_ros_message(
            source.velocity, destination.velocity);
        destination.visible = source.visible;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rclcpp::TypeAdapter<RJ::Time, builtin_interfaces::msg::Time>::convert_to_custom(
            source.stamp, destination.timestamp);
        rclcpp::TypeAdapter<rj_geometry::Pose, rj_geometry_msgs::msg::Pose>::convert_to_custom(
            source.pose, destination.pose);
        rclcpp::TypeAdapter<rj_geometry::Twist, rj_geometry_msgs::msg::Twist>::convert_to_custom(
            source.velocity, destination.velocity);
        destination.visible = source.visible;
    }
};

template <>
struct TypeAdapter<BallState, BallState::Msg> {
    using is_specialized = std::true_type;
    using custom_type = BallState;
    using ros_message_type = BallState::Msg;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rclcpp::TypeAdapter<RJ::Time, builtin_interfaces::msg::Time>::convert_to_ros_message(
            source.timestamp, destination.stamp);
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_ros_message(
            source.velocity, destination.velocity);
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_ros_message(
            source.position, destination.position);
        destination.visible = source.visible;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rclcpp::TypeAdapter<RJ::Time, builtin_interfaces::msg::Time>::convert_to_custom(
            source.stamp, destination.timestamp);
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
            source.velocity, destination.velocity);
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
            source.position, destination.position);
        destination.visible = source.visible;
    }
};

template <>
struct TypeAdapter<WorldState, WorldState::Msg> {
    using is_specialized = std::true_type;
    using custom_type = WorldState;
    using ros_message_type = WorldState::Msg;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rclcpp::TypeAdapter<BallState, BallState::Msg>::convert_to_ros_message(source.ball,
                                                                               destination.ball);
        destination.our_robots.clear();
        destination.our_robots.reserve(source.our_robots.size());
        for (const auto& robot : source.our_robots) {
            RobotState::Msg robot_msg;
            rclcpp::TypeAdapter<RobotState, RobotState::Msg>::convert_to_ros_message(robot,
                                                                                     robot_msg);
            destination.our_robots.push_back(std::move(robot_msg));
        }
        destination.their_robots.clear();
        destination.their_robots.reserve(source.their_robots.size());
        for (const auto& robot : source.their_robots) {
            RobotState::Msg robot_msg;
            rclcpp::TypeAdapter<RobotState, RobotState::Msg>::convert_to_ros_message(robot,
                                                                                     robot_msg);
            destination.their_robots.push_back(std::move(robot_msg));
        }
        rclcpp::TypeAdapter<RJ::Time, builtin_interfaces::msg::Time>::convert_to_ros_message(
            source.last_updated_time, destination.last_update_time);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rclcpp::TypeAdapter<BallState, BallState::Msg>::convert_to_custom(source.ball,
                                                                          destination.ball);
        destination.our_robots.clear();
        destination.our_robots.reserve(source.our_robots.size());
        for (const auto& robot_msg : source.our_robots) {
            RobotState robot;
            rclcpp::TypeAdapter<RobotState, RobotState::Msg>::convert_to_custom(robot_msg, robot);
            destination.our_robots.push_back(std::move(robot));
        }
        destination.their_robots.clear();
        destination.their_robots.reserve(source.their_robots.size());
        for (const auto& robot_msg : source.their_robots) {
            RobotState robot;
            rclcpp::TypeAdapter<RobotState, RobotState::Msg>::convert_to_custom(robot_msg, robot);
            destination.their_robots.push_back(std::move(robot));
        }
        rclcpp::TypeAdapter<RJ::Time, builtin_interfaces::msg::Time>::convert_to_custom(
            source.last_update_time, destination.last_updated_time);
    }
};

}  // namespace rclcpp
