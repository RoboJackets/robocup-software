#pragma once

#include <config_client/async_config_client.h>
#include <ros2_temp/async_message_queue.h>

#include <WorldState.hpp>
#include <atomic>
#include <mutex>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_vision_filter/camera/CameraFrame.hpp>
#include <rj_vision_filter/camera/World.hpp>
#include <thread>
#include <vector>

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
class VisionFilter {
public:
    /**
     * Starts a worker thread to do the vision processing
     */
    VisionFilter(Context* context);
    ~VisionFilter();

    void run();

    /**
     * Fills system state with the ball pos/vel
     *
     * @param state Current system state pointer
     */
    void fillBallState(WorldState* state);

    /**
     * Fills system state with the robots pos/vel
     *
     * @param state Current system state pointer
     * @param usBlue True if we are blue
     */
    void fillRobotState(WorldState* state, bool usBlue);

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
        const bool defend_plus_x =
            config_client_->config_client_node().gameSettings().defend_plus_x;
        return defend_plus_x ? -M_PI_2 : M_PI_2;
    }

    /**
     * @brief Returns the transform from the world to the team.
     * @return The transform from the world to the team frame.
     */
    [[nodiscard]] Geometry2d::TransformMatrix WorldToTeam() const {
        Geometry2d::TransformMatrix world_to_team =
            Geometry2d::TransformMatrix::translate(
                0,
                config_client_->config_client_node().fieldDimensions().length /
                    2.0f);
        world_to_team *= Geometry2d::TransformMatrix::rotate(
            static_cast<float>(TeamAngle()));
        return world_to_team;
    }

    std::thread worker;
    std::mutex worldLock;
    World world;

    std::atomic_bool threadEnd{};

    std::vector<CameraFrame> frameBuffer{};
    Context* context_;
    using DetectionFrameMsg = rj_msgs::msg::DetectionFrame;
    using AsyncDetectionFrameQueue =
        ros2_temp::AsyncMessageQueue<DetectionFrameMsg,
                                     ros2_temp::MessagePolicy::kQueue>;
    AsyncDetectionFrameQueue::UniquePtr detection_frame_queue_;
    config_client::AsyncConfigClient::UniquePtr config_client_;
    RJ::Time last_update_time_;
};