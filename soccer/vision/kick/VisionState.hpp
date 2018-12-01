#pragma once

#include <vector>
#include <Utils.hpp>

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"

class VisionState {
public:
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