#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <config_client/config_client.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <rj_topic_utils/message_queue.hpp>
#include <rj_utils/concurrent_queue.hpp>

#include "rj_vision_filter/camera/camera_frame.hpp"
#include "rj_vision_filter/camera/world.hpp"

namespace vision_filter {
using TeamColorMsg = rj_msgs::msg::TeamColor;

/**
 * Filters the vision measurements into a smoother velocity/position estimate for both the ball and
 * robots.
 */
class VisionFilter : public rclcpp::Node {
public:
    using WorldStateMsg = rj_msgs::msg::WorldState;
    using RobotStateMsg = rj_msgs::msg::RobotState;
    using BallStateMsg = rj_msgs::msg::BallState;

    /**
     * Initialize the vision filter and all callbacks.
     */
    VisionFilter(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief Creates a WorldStateMsg from the robot and ball Kalman filters.
     * @param us_blue True if we are blue.
     * @return The WorldStateMsg corresponding to the current VisionFilter
     * state.
     */
    WorldStateMsg build_world_state_msg(bool us_blue) const;

    /**
     * @brief Creates a BallStateMsg from the ball Kalman filter.
     * @return The BallStateMsg corresponding to the current VisionFilter
     * state.
     */
    BallStateMsg build_ball_state_msg() const;

    /**
     * @brief Creates a vector of RobotStateMsgs from the robot Kalman filters.
     * @param blue_team Uses the blue team's robots if true. Otherwise uses the
     * yellow team's robots.
     * @return A vector of  RobotStateMsg corresponding to blue_team.
     */
    std::vector<RobotStateMsg> build_robot_state_msgs(bool blue_team) const;

    /**
     * @brief Publishes the current state of the balls and robots.
     */
    void publish_state();

    // TODO(1562): (It's horrible, but it's only temporary until VisionFilter
    // gets refactored).
    /**
     * @brief Returns the team angle
     * @return The team angle.
     */
    [[nodiscard]] double team_angle() const {
        const bool defend_plus_x = config_client_.game_settings().defend_plus_x;
        return defend_plus_x ? -M_PI_2 : M_PI_2;
    }

    /**
     * @brief Returns the transform from the world to the team.
     * @return The transform from the world to the team frame.
     */
    [[nodiscard]] rj_geometry::TransformMatrix world_to_team() const {
        rj_geometry::TransformMatrix world_to_team = rj_geometry::TransformMatrix::translate(
            0, config_client_.field_dimensions().length / 2.0f);
        world_to_team *= rj_geometry::TransformMatrix::rotate(static_cast<float>(team_angle()));
        return world_to_team;
    }

    /**
     * @brief State of the world, ie. robots and ball.
     */
    World world_;

    using TeamColorMsgQueue =
        rj_topic_utils::MessageQueue<TeamColorMsg, rj_topic_utils::MessagePolicy::kLatest>;

    config_client::ConfigClient config_client_;

    /**
     * @brief Message Queue for TeamColor that takes the latest message.
     */
    TeamColorMsgQueue team_color_queue_;

    /**
     * @brief Timer driving regular publication.
     */
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::Subscription<DetectionFrameMsg>::SharedPtr detection_frame_sub_;

    /**
     * @brief Publisher for WorldStateMsg.
     */
    rclcpp::Publisher<WorldStateMsg>::SharedPtr world_state_pub_;

    ::params::LocalROS2ParamProvider param_provider_;
};
}  // namespace vision_filter