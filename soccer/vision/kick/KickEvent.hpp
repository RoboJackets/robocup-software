#pragma once

#include <vector>
#include <deque>

#include <Utils.hpp>

#include "vision/robot/WorldRobot.hpp"
#include "vision/ball/WorldBall.hpp"
#include "VisionState.hpp"

class KickEvent {
public:
    KickEvent() : isValid(false) {};
    KickEvent(RJ::Time kickTime, WorldRobot kickingRobot,
              std::deque<VisionState> statesSinceKick) :
              isValid(true), kickTime(kickTime),
              kickingRobot(kickingRobot),
              statesSinceKick(statesSinceKick) {}

    void addState(RJ::Time calcTime, WorldBall ball,
                  std::vector<WorldRobot> yellowRobots,
                  std::vector<WorldRobot> blueRobots);

    bool getIsValid();

    bool isValid;
    RJ::Time kickTime;
    WorldRobot kickingRobot;
    std::deque<VisionState> statesSinceKick;
};