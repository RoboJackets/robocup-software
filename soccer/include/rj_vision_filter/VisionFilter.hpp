#pragma once

#include <config_client/config_client.h>
#include <rj_param_utils/ros2_param_provider.h>
#include <rj_topic_utils/message_queue.h>

#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_utils/concurrent_queue.hpp>
#include <rj_vision_filter/camera/CameraFrame.hpp>
#include <rj_vision_filter/camera/World.hpp>
#include <thread>
#include <vector>

namespace vision_filter {
using TeamColorMsg = rj_msgs::msg::TeamColor;

/**
 * Uses a separate thread to filter the vision measurements into
 * a smoother velocity/position estimate for both the ball and robots.
 *
 * Add vision frames directly into the filter call the fill states functions
 * to push the newest estimates directly into the system state.
 *
 * Note: There may be a 1 frame delay between the measurements being added
 * and the measurements being included in the filter estimate.
 */
class VisionFilter : public rclcpp::Node {
public:
    using WorldStateMsg = rj_msgs::msg::WorldState;
    using RobotStateMsg = rj_msgs::msg::RobotState;
    using BallStateMsg = rj_msgs::msg::BallState;

    /**
     * Starts a worker thread to do the vision processing
     */
    VisionFilter(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief Runs prediction on all the Kalman Filters by calling out to
     * PredictStatesImpl. Also prints a warning if predict isn't running fast
     * enough.
     */
    void PredictStates();

    /**
     * @brief Runs prediction on all the Kalman Filters. For now, also performs
     * the update / merge step of the Kalman filters by emptying
     * detection_frame_queue_;
     */
    void PredictStatesImpl();

    /**
     * @brief Creates a WorldStateMsg from the robot and ball Kalman filters.
     * @param us_blue True if we are blue.
     * @return The WorldStateMsg corresponding to the current VisionFilter
     * state.
     */
    WorldStateMsg BuildWorldStateMsg(bool us_blue) const;

    /**
     * @brief Creates a BallStateMsg from the ball Kalman filter.
     * @return The BallStateMsg corresponding to the current VisionFilter
     * state.
     */
    BallStateMsg BuildBallStateMsg() const;

    /**
     * @brief Creates a vector of RobotStateMsgs from the robot Kalman filters.
     * @param blue_team Uses the blue team's robots if true. Otherwise uses the
     * yellow team's robots.
     * @return A vector of  RobotStateMsg corresponding to blue_team.
     */
    std::vector<RobotStateMsg> BuildRobotStateMsgs(bool blue_team) const;

    /**
     * @brief Publishes the current state of the balls and robots.
     */
    void PublishState();

    /**
     * @brief Obtains a std::vector<DetectionFrameMsg::UniqePtr> from
     * detection_frame_sub_, then converts that to std::vector<CameraFrame>
     * and stores it in new_frames_.
     */
    void GetFrames();

    // TODO(1562): (It's horrible, but it's only temporary until VisionFilter
    // gets refactored).
    /**
     * @brief Returns the team angle
     * @return The team angle.
     */
    [[nodiscard]] double TeamAngle() const {
        const bool defend_plus_x = config_client_.gameSettings().defend_plus_x;
        return defend_plus_x ? -M_PI_2 : M_PI_2;
    }

    /**
     * @brief Returns the transform from the world to the team.
     * @return The transform from the world to the team frame.
     */
    [[nodiscard]] Geometry2d::TransformMatrix WorldToTeam() const {
        Geometry2d::TransformMatrix world_to_team =
            Geometry2d::TransformMatrix::translate(
                0, config_client_.fieldDimensions().length / 2.0f);
        world_to_team *= Geometry2d::TransformMatrix::rotate(
            static_cast<float>(TeamAngle()));
        return world_to_team;
    }

    /**
     * @brief State of the world, ie. robots and ball.
     */
    World world;

    std::vector<CameraFrame> frameBuffer{};

    using TeamColorMsgQueue =
        rj_topic_utils::MessageQueue<TeamColorMsg,
                                     rj_topic_utils::MessagePolicy::kLatest>;
    using TimeMsg = builtin_interfaces::msg::Time;

    config_client::ConfigClient config_client_;

    /**
     * @brief Message Queue for TeamColor that takes the latest message.
     */
    TeamColorMsgQueue team_color_queue_;

    /**
     * @brief Timer driving regular predictions.
     */
    rclcpp::TimerBase::SharedPtr predict_timer_;

    rclcpp::Subscription<DetectionFrameMsg>::SharedPtr detection_frame_sub_;
    rj_utils::ConcurrentQueue<DetectionFrameMsg::UniquePtr>
        detection_frame_queue_;

    /**
     * @brief Publisher for WorldStateMsg.
     */
    rclcpp::Publisher<WorldStateMsg>::SharedPtr world_state_pub_;

    params::ROS2ParamProvider param_provider_;
};
}  // namespace vision_filter