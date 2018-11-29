#pragma once

#include <deque>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "vision/kick/KickEvent.hpp"
#include "vision/kick/VisionState.hpp"

/**
 * Detects extremely fast kicks in the case where the
 * 5 or more samples required by the slow kick detector
 * would take too long to collect and still have time to
 * react to the ball.
 *
 * Uses a very simple velocity change over 3 samples to test
 * for kicks.
 */
class FastKickDetector {
public:
    FastKickDetector();

    bool addRecord(RJ::Time calcTime, WorldBall ball,
                   std::vector<WorldRobot> yellowRobots,
                   std::vector<WorldRobot> blueRobots,
                   KickEvent& kickEvent);

private:
    bool detectKick();
    WorldRobot getClosestRobot();

    std::deque<VisionState> stateHistory;
};