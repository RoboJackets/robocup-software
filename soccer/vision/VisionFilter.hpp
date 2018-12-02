#pragma once

#include <mutex>

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
    void addFrames();
    void getBall();
    void getRobots();

private:
    static void workerThread();

    std::deque<CameraFrame> frameBuffer;
    World world;
};