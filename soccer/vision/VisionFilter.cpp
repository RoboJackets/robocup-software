#include "VisionFilter.hpp"

#include <iostream>

#include <Constants.hpp>
#include <Robot.hpp>

#include "vision/util/VisionFilterConfig.hpp"

VisionFilter::VisionFilter() {
    threadEnd.store(false, std::memory_order::memory_order_seq_cst);
    
    // Have to be careful so the entire initialization list
    // is created before the thread starts
    worker = std::thread(&VisionFilter::updateLoop, this);
}

VisionFilter::~VisionFilter() {
    // Signal end of thread
    threadEnd.store(true, std::memory_order::memory_order_seq_cst);

    // Wait for it to die
    worker.join();
}

void VisionFilter::addFrames(const std::vector<CameraFrame>& frames) {
    std::lock_guard<std::mutex> lock(frameLock);
    frameBuffer.insert(frameBuffer.end(), frames.begin(), frames.end());
}

void VisionFilter::fillBallState(WorldState* state) {
    std::lock_guard<std::mutex> lock(worldLock);
    const WorldBall& wb = world.getWorldBall();

    if (wb.getIsValid()) {
        state->ball.visible = true;
        state->ball.position = wb.getPos();
        state->ball.velocity = wb.getVel();
        state->ball.timestamp = wb.getTime();
    } else {
        state->ball.visible = false;
    }
}

void VisionFilter::fillRobotState(WorldState* state, bool usBlue) {
    std::lock_guard<std::mutex> lock(worldLock);
    const auto& ourWorldRobot = usBlue ? world.getRobotsBlue() : world.getRobotsYellow();
    const auto& oppWorldRobot = usBlue ? world.getRobotsYellow() : world.getRobotsBlue();

    // Fill our robots
    for (int i = 0; i < Num_Shells; i++) {
        const WorldRobot& wr = ourWorldRobot.at(i);

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();

        if (wr.getIsValid()) {
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity =
                Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        state->our_robots.at(i) = robot_state;
    }

    // Fill opp robots
    for (int i = 0; i < Num_Shells; i++) {
        const WorldRobot& wr = oppWorldRobot.at(i);

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();

        if (wr.getIsValid()) {
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity =
                Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        state->their_robots.at(i) = robot_state;
    }
}

void VisionFilter::updateLoop() {
    while (!threadEnd.load(std::memory_order::memory_order_seq_cst)) {
        RJ::Time start = RJ::now();

        {
            // Do update with whatever is in frame buffer
            std::lock_guard<std::mutex> lock1(frameLock);
            {
                std::lock_guard<std::mutex> lock2(worldLock);

                if (!frameBuffer.empty()) {
                    world.updateWithCameraFrame(RJ::now(), frameBuffer);
                    frameBuffer.clear();
                } else {
                    world.updateWithoutCameraFrame(RJ::now());
                }
            }
        }

        // Wait for the correct loop timings
        RJ::Seconds diff = RJ::now() - start;
        RJ::Seconds sleepLeft = RJ::Seconds(*VisionFilterConfig::vision_loop_dt) - diff;

        if (diff > RJ::Seconds(0)) {
            std::this_thread::sleep_for(sleepLeft);
        } else {
            std::cout << "WARNING : Filter is not running fast enough" << std::endl;
        }
    }
}