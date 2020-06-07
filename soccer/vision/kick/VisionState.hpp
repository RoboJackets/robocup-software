#pragma once

#include <utils.h>

#include <vector>

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"

/**
 * Snapshot of the state of all objects in vision at a specific time
 */
class VisionState {
public:
    /**
     * @param calcTime Current frame time
     * @param ball Current frame ball
     * @param yellowRobots Current frame robot list
     * @param blueRobots Current frame robot list
     */
    VisionState(RJ::Time calcTime, WorldBall ball,
                std::vector<WorldRobot> yellowRobots,
                std::vector<WorldRobot> blueRobots) :
                calcTime(calcTime), ball(ball),
                yellowRobots(yellowRobots),
                blueRobots(blueRobots) {}

    RJ::Time calcTime;
    WorldBall ball;
    std::vector<WorldRobot> yellowRobots;
    std::vector<WorldRobot> blueRobots;
};
