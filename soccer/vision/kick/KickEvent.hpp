#pragma once

#include <vector>
#include <deque>

#include <Utils.hpp>

#include "vision/robot/WorldRobot.hpp"
#include "vision/ball/WorldBall.hpp"
#include "VisionState.hpp"

class KickEvent {
public:
    KickEvent(RJ::Time kickTime, WorldRobot kickingRobot,
              std::deque<VisionState> statesSinceKick) :
              kickTime(kickTime), kickingRobot(kickingRobot),
              statesSinceKick(statesSinceKick) {}

    void addState(RJ::Time calcTime, WorldBall ball,
                  std::vector<WorldRobot> yellowRobots,
                  std::vector<WroldRobot> blueRobots);

    RJ::Time kickTime;
    WorldRobot kickingRobot;
    std::deque<VisionState> statesSinceKick;
};