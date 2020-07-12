#pragma once

#include <config_client/config_client.h>
#include <rj_param_utils/ros2_param_provider.h>

#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_utils/concurrent_queue.hpp>
#include <rj_vision_filter/camera/CameraFrame.hpp>
#include <rj_vision_filter/camera/World.hpp>
#include <thread>
#include <vector>

namespace vision_filter {
/**
 * Uses a seperate thread to filter the vision measurements into
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

    /**
     * @brief Returns the latest timestamp of the received vision packets.
     * @return
     */
    [[nodiscard]] RJ::Time GetLastVisionTime() const {
        return last_update_time_;
    }

private:
    /**
     * @brief Runs prediction on all the Kalman Filters. For now, also performs
     * the update step of the Kalman filter by emptying detection_frame_queue_;
     */
    void PredictStates();

    /**
     * @brief Creates a WorldStateMsg from the robot and ball Kalman filters.
     * @return The WorldStateMsg corresponding to the current VisionFilter
     * state.
     */
    WorldStateMsg BuildWorldStateMsg();

    /**
     * @brief Creates a BallStateMsg from the ball Kalman filter.
     * @return The BallStateMsg corresponding to the current VisionFilter
     * state.
     */
    BallStateMsg BuildBallStateMsg();

    /**
     * @brief Creates a vector of RobotStateMsgs from the robot Kalman filters.
     * @param blue_team Uses the blue team's robots if true. Otherwise uses the
     * yellow team's robots.
     * @return A vector of  RobotStateMsg corresponding to blue_team.
     */
    std::vector<RobotStateMsg> BuildRobotStateMsgs(bool blue_team);

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

    /**
     * @brief Returns the team angle (It's horrible, but it's only temporary
     * until VisionFilter gets refactored).
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

    std::mutex worldLock;
    World world;

    std::vector<CameraFrame> frameBuffer{};
    RJ::Time last_update_time_;

    rclcpp::CallbackGroup::SharedPtr publish_timer_callback_group_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::CallbackGroup::SharedPtr predict_timer_callback_group_;
    rclcpp::TimerBase::SharedPtr predict_timer_;

    rclcpp::CallbackGroup::SharedPtr update_callback_group_;
    rclcpp::Subscription<DetectionFrameMsg>::SharedPtr detection_frame_sub_;
    rj_utils::ConcurrentQueue<DetectionFrameMsg::UniquePtr>
        detection_frame_queue_;
    std::optional<RJ::Time> last_predict_time_;

    rclcpp::Publisher<WorldStateMsg>::SharedPtr world_state_pub_;

    config_client::ConfigClient config_client_;

    params::ROS2ParamProvider param_provider_;
};
}  // namespace vision_filter