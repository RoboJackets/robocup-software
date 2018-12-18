#pragma once

#include <mutex>
#include <vector>
#include <list>
#include <thread>

#include <SystemState.hpp>

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
     */
    void addFrames(std::vector<CameraFrame>& frames);
    void fillBallState(SystemState* state);
    void fillRobotState(SystemState* state, bool usBlue);

private:
    void workerThread();

    std::thread worker;

    std::mutex worldLock;
    World world;

    std::mutex threadEndLock;
    bool threadEnd;

    std::mutex frameLock;
    std::list<CameraFrame> frameBuffer;
};