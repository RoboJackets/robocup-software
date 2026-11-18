#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/game_settings.hpp>
#include <rj_geometry/transform_matrix.hpp>
#include <rj_referee_msgs/msg/game_settings.hpp>
#include <rj_referee_msgs/msg/team_color.hpp>
#include <rj_vision_msgs/msg/detection_frame.hpp>
#include <rj_vision_msgs/msg/field_dimensions.hpp>
#include <rj_vision_msgs/msg/world_state.hpp>

#include "rj_vision_filter/camera/camera_frame.hpp"
#include "rj_vision_filter/camera/world.hpp"
#include "rj_vision_filter/params.hpp"

namespace vision_filter {
using TeamColorMsg = rj_referee_msgs::msg::TeamColor;

/**
 * Filters the vision measurements into a smoother velocity/position estimate for both the ball and
 * robots.
 */
class VisionFilter : public rclcpp::Node {
public:
    using WorldStateMsg = rj_vision_msgs::msg::WorldState;
    using RobotStateMsg = rj_vision_msgs::msg::RobotState;
    using BallStateMsg = rj_vision_msgs::msg::BallState;

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
    double team_angle();

    /**
     * @brief Returns the transform from the world to the team.
     * @return The transform from the world to the team frame.
     */
    rj_geometry::TransformMatrix world_to_team();

    /**
     * @brief Vision filter parameters shared with world_ and everything it
     * owns. Declared before world_ so they are ready when its constructor runs.
     */
    std::shared_ptr<VisionFilterParams> params_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    /**
     * @brief State of the world, ie. robots and ball.
     */
    World world_;

    rclcpp::Subscription<TeamColorMsg>::SharedPtr team_color_sub_;
    bool is_blue_;

    /**
     * @brief The current game settings
     */
    GameSettings::Msg game_settings_{};

    /**
     * Whether or not we have up-to-date game settings
     */
    bool has_game_settings_ = false;

    /**
     * @brief The field dimensions
     */
    FieldDimensions::Msg field_dimensions_{};

    /**
     * Whether or not we've recieved the field dimensions from the vision filter
     */
    bool has_field_dimensions_ = false;

    /**
     * @brief Timer driving regular publication.
     */
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::Subscription<DetectionFrameMsg>::SharedPtr detection_frame_sub_;

    /**
     * @brief Publisher for WorldStateMsg.
     */
    rclcpp::Publisher<WorldStateMsg>::SharedPtr world_state_pub_;

    /**
     * @brief Subscriber for GameSettings msgs
     */
    rclcpp::Subscription<GameSettings::Msg>::SharedPtr game_settings_sub_;

    /**
     * @brief Subscriber for FieldDimensions msgs
     */
    rclcpp::Subscription<FieldDimensions::Msg>::SharedPtr field_dimensions_sub_;
};
}  // namespace vision_filter
