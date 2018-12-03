#include "VisionFilter.hpp"

#include <iostream>

#include "vision/util/VisionFilterConfig.hpp"

VisionFilter::VisionFilter() : threadEnd(false) {
    // Have to be careful so the entire initialization list
    // is created before the thread starts
    worker = std::thread(&VisionFilter::workerThread, this);
}

VisionFilter::~VisionFilter() {
    // Signal end of thread
    threadEndLock.lock();
    threadEnd = true;
    threadEndLock.unlock();

    // Wait for it to die
    worker.join();
}

void VisionFilter::addFrames(std::vector<CameraFrame>& frames) {
    frameLock.lock();
    frameBuffer.insert(frameBuffer.end(), frames.begin(), frames.end());
    frameLock.unlock();
}

void VisionFilter::getBall() {
    worldLock.lock();
    worldLock.unlock();
}

void VisionFilter::getRobots() {
    worldLock.lock();
    worldLock.unlock();
}

void VisionFilter::workerThread() {
    while (true) {
        RJ::Time start = RJ::now();

        // Do update with whatever is in frame buffer
        frameLock.lock();
        worldLock.lock();
        if (frameBuffer.size() > 0) {
            world.updateWithCameraFrame(RJ::now(), frameBuffer);
        } else {
            world.updateWithoutCameraFrame(RJ::now());
        }
        worldLock.unlock();
        frameLock.unlock();


        // Wait for the correct loop timings
        RJ::Seconds diff = RJ::now() - start;
        RJ::Seconds sleepLeft = RJ::Seconds(*VisionFilterConfig::vision_loop_dt) - diff;

        if (diff > RJ::Seconds(0)) {
            std::this_thread::sleep_for(sleepLeft);
        } else {
            std::cout << "WARNING : Filter is not running fast enough" << std::endl;
        }

        // Make sure we shouldn't stop
        threadEndLock.lock();
        if (threadEnd) {
            break;
        }
        threadEndLock.unlock();
    }
}