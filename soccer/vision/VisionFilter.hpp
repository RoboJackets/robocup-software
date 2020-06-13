#pragma once

#include <WorldState.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "vision/camera/CameraFrame.hpp"
#include "vision/camera/World.hpp"

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
    VisionFilter();
    ~VisionFilter();

    /**
     * Adds a list of frames that arrived
     *
     * @param frames List of new frames
     */
    void addFrames(const std::vector<CameraFrame>& frames);

    /**
     * Fills system state with the ball pos/vel
     *
     * @param state Current system state pointer
     */
    void fillBallState(WorldState& state);

    /**
     * Fills system state with the robots pos/vel
     *
     * @param state Current system state pointer
     * @param usBlue True if we are blue
     */
    void fillRobotState(WorldState& state, bool usBlue);

private:
    void updateLoop();

    std::thread worker;

    std::mutex worldLock;
    World world;

    std::atomic_bool threadEnd{};

    std::mutex frameLock;
    std::vector<CameraFrame> frameBuffer{};
};