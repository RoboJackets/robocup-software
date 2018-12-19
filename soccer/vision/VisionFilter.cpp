#include "VisionFilter.hpp"

#include <iostream>

#include <Constants.hpp>
#include <Robot.hpp>

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

void VisionFilter::fillBallState(SystemState* state) {
    worldLock.lock();
    WorldBall wb = world.getWorldBall();
    worldLock.unlock();

    if (wb.getIsValid()) {
        state->ball.valid = true;
        state->ball.pos = wb.getPos();
        state->ball.vel = wb.getVel();
        state->ball.time = wb.getTime();
    } else {
        state->ball.valid = false;
    }
}

void VisionFilter::fillRobotState(SystemState* state, bool usBlue) {
    worldLock.lock();
    std::vector<WorldRobot> yellowTeam = world.getRobotsYellow();
    std::vector<WorldRobot> blueTeam = world.getRobotsBlue();
    worldLock.unlock();

    // Fill our robots
    for (int i = 0; i < Num_Shells; i++) {
        OurRobot* robot = state->self.at(i);
        WorldRobot wr;

        if (usBlue) {
            wr = blueTeam.at(i);
        } else {
            wr = yellowTeam.at(i);
        }

        robot->visible = wr.getIsValid();
        robot->velValid = wr.getIsValid();

        if (wr.getIsValid()) {
            robot->pos = wr.getPos();
            robot->vel = wr.getVel();
            robot->angle = wr.getTheta();
            robot->angleVel = wr.getOmega();
            robot->time = wr.getTime();
        }
    }

    // Fill opp robots
    for (int i = 0; i < Num_Shells; i++) {
        OpponentRobot* robot = state->opp.at(i);
        WorldRobot wr;

        if (usBlue) {
            wr = yellowTeam.at(i);
        } else {
            wr = blueTeam.at(i);
        }

        robot->visible = wr.getIsValid();
        robot->velValid = wr.getIsValid();

        if (wr.getIsValid()) {
            robot->pos = wr.getPos();
            robot->vel = wr.getVel();
            robot->angle = wr.getTheta();
            robot->angleVel = wr.getOmega();
            robot->time = wr.getTime();
        }
    }
}

void VisionFilter::workerThread() {
    while (true) {
        RJ::Time start = RJ::now();

        // Do update with whatever is in frame buffer
        frameLock.lock();
        worldLock.lock();
        if (frameBuffer.size() > 0) {
            world.updateWithCameraFrame(RJ::now(), frameBuffer);
            frameBuffer.clear();
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