#pragma once

#include <rj_param_utils/ros2_param_provider.h>

#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
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
    /**
     * Starts a worker thread to do the vision processing
     */
    VisionFilter(const rclcpp::NodeOptions& options);
    ~VisionFilter();

    /**
     * Fills system state with the ball pos/vel
     *
     * @param state Current system state pointer
     */
    void fillBallState(SystemState& state);

    /**
     * Fills system state with the robots pos/vel
     *
     * @param state Current system state pointer
     * @param usBlue True if we are blue
     */
    void fillRobotState(SystemState& state, bool usBlue);

    /**
     * @brief Returns the latest timestamp of the received vision packets.
     * @return
     */
    [[nodiscard]] RJ::Time GetLastVisionTime() const {
        return last_update_time_;
    }

private:
    void updateLoop();

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

    std::thread worker;

    std::mutex worldLock;
    World world;

    std::atomic_bool threadEnd{};

    std::vector<CameraFrame> frameBuffer{};
    RJ::Time last_update_time_;

    rclcpp::CallbackGroup::SharedPtr publish_timer_callback_group_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::CallbackGroup::SharedPtr predict_timer_callback_group_;
    rclcpp::TimerBase::SharedPtr predict_timer_;

    params::ROS2ParamProvider param_provider_;
};
}  // namespace vision_filter