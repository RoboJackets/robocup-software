#pragma once

#include <mutex>
#include <vector>
#include <list>
#include <thread>

#include <SystemState.hpp>

#include "vision/camera/CameraFrame.hpp"
#include "vision/camera/World.hpp"

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