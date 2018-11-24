#pragma once

#include <deque>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "vision/kick/KickEvent.hpp"
#include "vision/kick/VisionState.hpp"

class SlowKickDetector {
public:
    SlowKickDetector();

    bool addRecord(RJ::Time calcTime, WorldBall ball,
                   std::vector<WorldRobot> yellowRobots,
                   std::vector<WorldRobot> blueRobots,
                   KickEvent& kickEvent);

private:
    bool process(KickEvent& kickEvent);
    bool backtrackToKick(KickEvent& kickEvent);

    bool checkAllValidators();

    bool distanceValidator();
    bool velocityValidator();
    bool distanceIncreasingValidator();
    bool inFrontValidator();

    std::deque<VisionState> stateHistory;
};