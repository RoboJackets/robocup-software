#pragma once

#include <deque>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "vision/kick/KickEvent.hpp"
#include "vision/kick/VisionState.hpp"

/**
 * Accruately detects kicks by robots using 5 or more samples
 * in history.
 *
 * Used in the general case when we have time to take a second
 * and double check results before sending it back
 */
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