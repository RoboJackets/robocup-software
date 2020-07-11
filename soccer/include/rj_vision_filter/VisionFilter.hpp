#pragma once

#include <ros2_temp/detection_frame_sub.h>

#include <WorldState.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <rj_vision_filter/camera/CameraFrame.hpp>
#include <rj_vision_filter/camera/World.hpp>

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

    std::thread worker;

    std::mutex worldLock;
    World world;

    std::atomic_bool threadEnd{};

    std::vector<CameraFrame> frameBuffer{};
    Context* context_;
    std::unique_ptr<ros2_temp::DetectionFrameSub> detection_frame_sub_;
    RJ::Time last_update_time_;
};